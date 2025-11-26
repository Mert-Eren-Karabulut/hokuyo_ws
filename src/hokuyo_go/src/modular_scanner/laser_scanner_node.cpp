#include "hokuyo_go/modular_scanner/laser_scanner_node.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sstream>

namespace hokuyo_go
{

    LaserScannerNode::LaserScannerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : nh_(nh), pnh_(pnh),
          tiltangle_(0.0), panangle_(0.0),
          joint_states_received_(false),
          last_tiltangle_for_velocity_(0.0),
          last_panangle_for_velocity_(0.0),
          path_speed_sum_(0.0),
          path_speed_sample_count_(0),
          scan_count_(0),
          scanning_active_(false),
          save_pointcloud_(false),
          scan_duration_(0.0),
          global_voxel_size_(0.05f)
    {
        // Get parameters
        double v_target = 1.0;
        double delta_1_deg = 30.0; // pan
        double delta_2_deg = 20.0; // tilt

        pnh_.param("target_velocity", v_target, 1.0);
        pnh_.param("scan_duration", scan_duration_, 0.0);
        pnh_.param("pan_limit_deg", delta_1_deg, 30.0);
        pnh_.param("tilt_limit_deg", delta_2_deg, 20.0);
        pnh_.param("voxel_size", global_voxel_size_, 0.05f);
        pnh_.param("subvoxel_threshold", SUBVOXEL_THRESHOLD, 0.35f);

        double delta_1 = delta_1_deg * PI / 180.0;
        double delta_2 = delta_2_deg * PI / 180.0;

        //initialize current pattern limits
        current_pan_limit_max_ = delta_1;
        current_pan_limit_min_ = -delta_1;
        current_tilt_limit_max_ = delta_2;
        current_tilt_limit_min_ = -delta_2;

        // Initialize components
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(30.0));
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        voxel_grid_ = std::make_unique<VoxelGrid>(global_voxel_size_);
        scanner_ = std::make_unique<ParametricScanner>(v_target, delta_1, delta_2);

        // Setup subscribers
        joint_sub_ = nh_.subscribe("/joint_states", 100, &LaserScannerNode::jointStateCallback, this);
        laser_sub_ = nh_.subscribe("/scan", 100, &LaserScannerNode::laserCallback, this);
        focus_point_sub_ = nh_.subscribe("/clicked_point", 10, &LaserScannerNode::focusPointCallback, this);

        // Setup publishers
        joint_cmd_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_command", 10);
        pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("debug_marker", 10);

        last_velocity_time_ = std::chrono::high_resolution_clock::now();

        ROS_INFO("=== Parametric Position-Based Laser Scanner with TF Voxelization ===");
        ROS_INFO("Target velocity: %.2f rad/s", v_target);
        ROS_INFO("Pan limit: ±%.1f degrees", delta_1_deg);
        ROS_INFO("Tilt limit: ±%.1f degrees", delta_2_deg);
        ROS_INFO("Scan duration: %.1f seconds", scan_duration_);
        ROS_INFO("Voxel size: %.0fmm | Subvoxel threshold: %.0f%% | Max levels: %d",
                 global_voxel_size_ * 1000.0f, SUBVOXEL_THRESHOLD * 100.0f, MAX_SUBVOXEL_LEVEL);
    }

    LaserScannerNode::~LaserScannerNode() {}

    void LaserScannerNode::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        // Extract laser data (from ranges[44] to ranges[724], total 681 points)
        laser_scan_ = std::vector<float>(msg->ranges.begin() + 44, msg->ranges.begin() + 725);
        last_laser_ = *msg;

        processLaserScan(msg);
    }

    void LaserScannerNode::processLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        if (!scanning_active_ || laser_scan_.empty() || !tf_buffer_ || !joint_states_received_)
            return;

        int scan_size = static_cast<int>(laser_scan_.size());
        const float angle_min = -2.094395102;
        const float angle_inc = 0.0061359232;

        for (int i = 0; i < scan_size; i++)
        {
            const float r = laser_scan_[i];
            if (!std::isfinite(r) || r <= 0.10 || r > 4.0)
                continue;

            float angle = angle_inc * i + angle_min;

            geometry_msgs::PointStamped pt_laser;
            pt_laser.header.frame_id = "laser";
            pt_laser.header.stamp = msg->header.stamp;
            pt_laser.point.x = r * std::cos(angle);
            pt_laser.point.y = r * std::sin(angle);
            pt_laser.point.z = 0.0;

            if (!tf_buffer_->canTransform("map", "laser", pt_laser.header.stamp, ros::Duration(0.02)))
                continue;

            try
            {
                geometry_msgs::PointStamped pt_map;
                tf_buffer_->transform(pt_laser, pt_map, "map", ros::Duration(0.02));

                voxel_grid_->insertPoint(pt_map.point.x, pt_map.point.y, pt_map.point.z, r);
            }
            catch (tf2::TransformException &ex)
            {
                continue;
            }
        }

        scan_count_++;

        // Progress reporting
        if (scan_count_ % 50 == 0)
        {
            std::vector<VoxelPoint> leaf_points = voxel_grid_->getPoints();
            int total_leaf_voxels = leaf_points.size();

            double avg_path_speed = 0.0;
            if (path_speed_sample_count_ > 0)
            {
                avg_path_speed = path_speed_sum_ / path_speed_sample_count_;
            }

            ROS_INFO("Scans: %d | Voxels: %zu | Leaf voxels: %d | Path: %.3f rad/s",
                     scan_count_, voxel_grid_->size(), total_leaf_voxels, avg_path_speed);

            path_speed_sum_ = 0.0;
            path_speed_sample_count_ = 0;
        }
    }

    void LaserScannerNode::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
    {
        if (msg->position.size() >= 2)
        {
            tiltangle_ = msg->position[0];
            panangle_ = msg->position[1];
            joint_states_received_ = true;

            // Calculate path-following speed
            auto current_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> dt = current_time - last_velocity_time_;

            if (dt.count() > 0.001)
            {
                double tilt_velocity = (tiltangle_ - last_tiltangle_for_velocity_) / dt.count();
                double pan_velocity = (panangle_ - last_panangle_for_velocity_) / dt.count();
                double path_speed = sqrt(tilt_velocity * tilt_velocity + pan_velocity * pan_velocity);

                path_speed_sum_ += path_speed;
                path_speed_sample_count_++;

                last_tiltangle_for_velocity_ = tiltangle_;
                last_panangle_for_velocity_ = panangle_;
                last_velocity_time_ = current_time;
            }
        }

        // Check if scan duration has elapsed
        if (scanning_active_ && scan_duration_ > 0.0)
        {
            auto current_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = current_time - scan_start_time_;

            if (elapsed.count() >= scan_duration_)
            {
                scanning_active_ = false;
                save_pointcloud_ = true;
                ROS_INFO("Scan complete after %.2f seconds", elapsed.count());
            }
        }
    }

    void LaserScannerNode::focusPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
    {
        ROS_INFO("Focus point received at (%.2f, %.2f, %.2f)",
                 msg->point.x, msg->point.y, msg->point.z);

        try
        {
            geometry_msgs::PointStamped pt_laser;
            if (!tf_buffer_->canTransform("laser", msg->header.frame_id, msg->header.stamp, ros::Duration(0.1)))
            {
                ROS_WARN("Cannot transform focus point to laser frame");
                return;
            }

            tf_buffer_->transform(*msg, pt_laser, "laser", ros::Duration(0.1));

            double lx = pt_laser.point.x;
            double ly = pt_laser.point.y;
            double lz = pt_laser.point.z;

            double range = sqrt(lx * lx + ly * ly + lz * lz);
            if (range < 0.01)
            {
                ROS_WARN("Target too close to laser origin");
                return;
            }

            double current_pan = panangle_;
            double current_tilt = tiltangle_;
            double delta_pan = atan2(ly, lx);
            double delta_tilt = atan2(lz, lx);

            double phi_solved = current_pan - delta_pan;
            double theta_solved = current_tilt + delta_tilt;

            bool target_behind = (lx < 0.0);
            if (target_behind)
            {
                theta_solved += PI;
            }

            // Normalize angles
            while (phi_solved > PI)
                phi_solved -= 2.0 * PI;
            while (phi_solved < -PI)
                phi_solved += 2.0 * PI;
            while (theta_solved > PI)
                theta_solved -= 2.0 * PI;
            while (theta_solved < -PI)
                theta_solved += 2.0 * PI;

            double delta_1, delta_2;
            scanner_->getPatternLimits(delta_1, delta_2);

            if (checkPatternLimits(phi_solved, theta_solved, delta_1, delta_2))
            {
                ROS_INFO("=== UPDATING SCAN CENTER ===");
                scanner_->setOffsets(phi_solved, theta_solved);
                ROS_INFO("New center: Pan=%.2f°, Tilt=%.2f°",
                         phi_solved * 180.0 / PI, theta_solved * 180.0 / PI);
                placeDebugPoint(msg->point.x, msg->point.y, msg->point.z, 255, 0, 0);
                calculateLocalizedSpeed();
            }
            else
            {
                ROS_WARN("Cannot center pattern - would exceed physical limits");
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform exception: %s", ex.what());
        }
    }

    float LaserScannerNode::calculateLocalizedSpeed()
    {
        // convert the scan boundaries(current_tilt_limit_max_, etc.) from spherical to cartesian.
        //this will form a frustum in 3D space representing the scan area. Then we need to calculate
        //the average point distance within this frustum with respect to origin.
        
        // Get all voxel points
        std::vector<VoxelPoint> voxel_points = voxel_grid_->getPoints();
        
        if (voxel_points.empty())
        {
            return 0.0f;
        }
        
        // Define frustum boundaries in spherical coordinates
        double pan_min = current_pan_limit_min_;
        double pan_max = current_pan_limit_max_;
        double tilt_min = current_tilt_limit_min_;
        double tilt_max = current_tilt_limit_max_;
        
        float distance_sum = 0.0f;
        int point_count = 0;
        
        // Track min/max coordinates for bounding box
        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();
        float max_z = std::numeric_limits<float>::lowest();
        
        // Iterate through all voxel points and check if they fall within the frustum
        for (const auto& point : voxel_points)
        {
            double x = point.x;
            double y = point.y;
            double z = point.z;
            
            // Calculate distance from origin
            double distance = sqrt(x * x + y * y + z * z);
            
            if (distance < 0.01)
                continue;
            
            // Convert cartesian to spherical coordinates
            double pan = atan2(y, x);
            double tilt = atan2(z, sqrt(x * x + y * y));
            
            // Normalize angles to [-PI, PI]
            while (pan > PI)
                pan -= 2.0 * PI;
            while (pan < -PI)
                pan += 2.0 * PI;
            while (tilt > PI)
                tilt -= 2.0 * PI;
            while (tilt < -PI)
                tilt += 2.0 * PI;
            
            // Check if point is within frustum boundaries
            if (pan >= pan_min && pan <= pan_max &&
                tilt >= tilt_min && tilt <= tilt_max)
            {
                distance_sum += distance;
                point_count++;
                
                // Update bounding box
                min_x = std::min(min_x, static_cast<float>(x));
                min_y = std::min(min_y, static_cast<float>(y));
                min_z = std::min(min_z, static_cast<float>(z));
                max_x = std::max(max_x, static_cast<float>(x));
                max_y = std::max(max_y, static_cast<float>(y));
                max_z = std::max(max_z, static_cast<float>(z));
            }
        }
        
        // Publish bounding box visualization
        if (point_count > 0)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "scan_bounding_box";
            marker.id = 1;
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.02; // Line width
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            marker.color.a = 0.8;

            // Define 8 corners of the bounding box
            geometry_msgs::Point corners[8];
            corners[0].x = min_x; corners[0].y = min_y; corners[0].z = min_z;
            corners[1].x = max_x; corners[1].y = min_y; corners[1].z = min_z;
            corners[2].x = max_x; corners[2].y = max_y; corners[2].z = min_z;
            corners[3].x = min_x; corners[3].y = max_y; corners[3].z = min_z;
            corners[4].x = min_x; corners[4].y = min_y; corners[4].z = max_z;
            corners[5].x = max_x; corners[5].y = min_y; corners[5].z = max_z;
            corners[6].x = max_x; corners[6].y = max_y; corners[6].z = max_z;
            corners[7].x = min_x; corners[7].y = max_y; corners[7].z = max_z;

            // Draw bottom face
            for (int i = 0; i < 4; i++)
            {
                marker.points.push_back(corners[i]);
                marker.points.push_back(corners[(i + 1) % 4]);
            }

            // Draw top face
            for (int i = 4; i < 8; i++)
            {
                marker.points.push_back(corners[i]);
                marker.points.push_back(corners[((i - 4) + 1) % 4 + 4]);
            }

            // Draw vertical edges
            for (int i = 0; i < 4; i++)
            {
                marker.points.push_back(corners[i]);
                marker.points.push_back(corners[i + 4]);
            }

            marker_pub_.publish(marker);
            
            return distance_sum / point_count;
        }
        
        return 0.0f;
    }

    bool LaserScannerNode::checkPatternLimits(double phi_offset, double theta_offset,
                                              double delta_1, double delta_2)
    {
        double pan_max = phi_offset + delta_1;
        double pan_min = phi_offset - delta_1;
        double tilt_max = theta_offset + delta_2;
        double tilt_min = theta_offset - delta_2;

        if (pan_max <= PAN_MAX && pan_min >= PAN_MIN &&
            tilt_max <= TILT_MAX && tilt_min >= TILT_MIN)
        {
            //set the current limits
            current_pan_limit_max_ = pan_max;
            current_pan_limit_min_ = pan_min;
            current_tilt_limit_max_ = tilt_max;
            current_tilt_limit_min_ = tilt_min;
            return true;
        }
        return false;
    }

    void LaserScannerNode::publishPointCloud()
    {
        std::vector<VoxelPoint> voxel_points = voxel_grid_->getPoints();

        if (voxel_points.empty())
            return;

        cloud_out_.header.frame_id = "map";
        cloud_out_.header.stamp = ros::Time::now();
        cloud_out_.height = 1;
        cloud_out_.width = voxel_points.size();
        cloud_out_.fields.resize(4);

        cloud_out_.fields[0].name = "x";
        cloud_out_.fields[0].offset = 0;
        cloud_out_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_out_.fields[0].count = 1;

        cloud_out_.fields[1].name = "y";
        cloud_out_.fields[1].offset = 4;
        cloud_out_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_out_.fields[1].count = 1;

        cloud_out_.fields[2].name = "z";
        cloud_out_.fields[2].offset = 8;
        cloud_out_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_out_.fields[2].count = 1;

        cloud_out_.fields[3].name = "rgb";
        cloud_out_.fields[3].offset = 12;
        cloud_out_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        cloud_out_.fields[3].count = 1;

        cloud_out_.point_step = 16;
        cloud_out_.row_step = cloud_out_.point_step * cloud_out_.width;
        cloud_out_.data.resize(cloud_out_.row_step * cloud_out_.height);
        cloud_out_.is_dense = false;

        for (size_t i = 0; i < voxel_points.size(); ++i)
        {
            float *pstep = (float *)&cloud_out_.data[i * cloud_out_.point_step];
            pstep[0] = voxel_points[i].x;
            pstep[1] = voxel_points[i].y;
            pstep[2] = voxel_points[i].z;

            uint8_t r, g, b;
            voxel_points[i].getColor(r, g, b);

            uint32_t rgb_packed = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
            pstep[3] = *reinterpret_cast<float *>(&rgb_packed);
        }

        pcl_pub_.publish(cloud_out_);
    }

    void LaserScannerNode::publishJointCommand(double tilt_target, double pan_target)
    {
        sensor_msgs::JointState joint_cmd;
        joint_cmd.header.stamp = ros::Time::now();
        joint_cmd.name.push_back("joint1");
        joint_cmd.name.push_back("joint2");
        joint_cmd.position.resize(2);
        joint_cmd.position[0] = tilt_target;
        joint_cmd.position[1] = pan_target;
        joint_cmd_pub_.publish(joint_cmd);
    }

    void LaserScannerNode::savePointCloud()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_out_, *pcl_cloud);

        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "pointcloud_" << timestamp << ".pcd";

        pcl::io::savePCDFileASCII(ss.str(), *pcl_cloud);
        ROS_INFO("PointCloud saved to %s (%zu points)", ss.str().c_str(), pcl_cloud->size());

        voxel_grid_->clear();
        scan_count_ = 0;
        ROS_INFO("Ready for next scan. Waiting at home position...");
    }

    void LaserScannerNode::placeDebugPoint(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "debug_points";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = r / 255.0;
        marker.color.g = g / 255.0;
        marker.color.b = b / 255.0;
        marker.color.a = 1.0;

        marker_pub_.publish(marker);
    }

    void LaserScannerNode::run()
    {
        ros::Rate loop(50); // 50Hz control loop

        // Wait for initial joint state
        ROS_INFO("Waiting for joint states from Arduino...");
        while (ros::ok() && !joint_states_received_)
        {
            ros::spinOnce();
            loop.sleep();
        }

        ROS_INFO("Joint states received. Waiting for TF transforms...");
        ros::Duration(2.0).sleep();

        // Start scanning
        scanning_active_ = true;
        scan_start_time_ = std::chrono::high_resolution_clock::now();
        ROS_INFO("=== SCANNING STARTED ===");

        while (ros::ok())
        {
            double pan_target, tilt_target;

            if (scanning_active_)
            {
                scanner_->updateParameterTime();
                scanner_->getTargetAngles(pan_target, tilt_target);
            }
            else
            {
                // Return to home position
                pan_target = 0.0;
                tilt_target = 0.0;
            }

            publishJointCommand(tilt_target, pan_target);
            publishPointCloud();

            if (save_pointcloud_)
            {
                savePointCloud();
                save_pointcloud_ = false;
            }

            ros::spinOnce();
            loop.sleep();
        }
    }

} // namespace hokuyo_go
