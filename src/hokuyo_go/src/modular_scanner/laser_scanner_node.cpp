#include "hokuyo_go/modular_scanner/laser_scanner_node.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sstream>
#include <algorithm>
#include <limits>

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
          global_voxel_size_(0.05f),
          observation_phase_active_(true),
          focused_scanning_active_(false),
          current_interval_idx_(0),
          current_zone_idx_(0)
    {
        // Get parameters
        double v_target = 1.0;
        double delta_1_deg = 30.0; // pan (normal operation)
        double delta_2_deg = 20.0; // tilt (normal operation)
        double delta_1_observe_deg = 60.0; // pan (observation phase)
        double delta_2_observe_deg = 40.0; // tilt (observation phase)

        pnh_.param("target_velocity", v_target, 1.0);
        pnh_.param("scan_duration", scan_duration_, 0.0);
        pnh_.param("pan_limit_deg", delta_1_deg, 30.0);
        pnh_.param("tilt_limit_deg", delta_2_deg, 20.0);
        pnh_.param("pan_observe_limit_deg", delta_1_observe_deg, 60.0);
        pnh_.param("tilt_observe_limit_deg", delta_2_observe_deg, 40.0);
        pnh_.param("voxel_size", global_voxel_size_, 0.05f);
        pnh_.param("subvoxel_threshold", SUBVOXEL_THRESHOLD, 0.35f);

        double delta_1 = delta_1_deg * PI / 180.0;
        double delta_2 = delta_2_deg * PI / 180.0;
        
        // Store observation deltas in radians
        delta_1_observe_ = delta_1_observe_deg * PI / 180.0;
        delta_2_observe_ = delta_2_observe_deg * PI / 180.0;
        

        //initialize current pattern limits
        current_pan_limit_max_ = delta_1;
        current_pan_limit_min_ = -delta_1;
        current_tilt_limit_max_ = delta_2;
        current_tilt_limit_min_ = -delta_2;

        // Initialize components
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(30.0));
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        voxel_grid_ = std::make_unique<VoxelGrid>(global_voxel_size_);
        
        // Initialize scanner with OBSERVATION deltas first (wider scan)
        scanner_ = std::make_unique<ParametricScanner>(v_target, delta_1_observe_, delta_2_observe_);

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
        ROS_INFO("Normal mode - Pan limit: ±%.1f degrees, Tilt limit: ±%.1f degrees", delta_1_deg, delta_2_deg);
        ROS_INFO("Observation mode - Pan limit: ±%.1f degrees, Tilt limit: ±%.1f degrees", delta_1_observe_deg, delta_2_observe_deg);
        ROS_INFO("Observation duration: %.1f seconds", OBSERVATION_DURATION);
        ROS_INFO("Histogram min distance: %.1f meters", HISTOGRAM_MIN_DISTANCE);
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

    void LaserScannerNode::storeSphericalPoint(float x, float y, float z)
    {
        // Calculate spherical coordinates from cartesian
        float r = sqrt(x * x + y * y + z * z);
        
        // Only store points beyond minimum distance for histogram
        if (r < HISTOGRAM_MIN_DISTANCE)
            return;
        
        float pan = atan2(y, x);
        float tilt = atan2(z, sqrt(x * x + y * y));
        
        SphericalPoint sp;
        sp.r = r;
        sp.pan = pan;
        sp.tilt = tilt;
        
        spherical_points_.push_back(sp);
    }

    void LaserScannerNode::convertVoxelsToSpherical()
    {
        // Get voxelized points from the voxel grid
        std::vector<VoxelPoint> voxel_points = voxel_grid_->getPoints();
        
        ROS_INFO("Converting %zu voxelized points to spherical coordinates...", voxel_points.size());
        
        // Clear any existing spherical points
        spherical_points_.clear();
        
        // Convert each voxelized point to spherical coordinates
        for (const auto& vp : voxel_points)
        {
            storeSphericalPoint(vp.x, vp.y, vp.z);
        }
        
        ROS_INFO("Stored %zu spherical points (r >= %.1fm)", 
                 spherical_points_.size(), HISTOGRAM_MIN_DISTANCE);
    }

    void LaserScannerNode::computeDistanceHistogram()
    {
        if (spherical_points_.empty())
        {
            ROS_WARN("No spherical points collected for histogram computation");
            return;
        }
        
        // Find min and max r values
        float r_min = std::numeric_limits<float>::max();
        float r_max = std::numeric_limits<float>::lowest();
        
        for (const auto& sp : spherical_points_)
        {
            r_min = std::min(r_min, sp.r);
            r_max = std::max(r_max, sp.r);
        }
        
        // Create histogram bins
        float bin_width = (r_max - r_min) / NUM_HISTOGRAM_BINS;
        distance_histogram_.clear();
        distance_histogram_.resize(NUM_HISTOGRAM_BINS);
        
        for (int i = 0; i < NUM_HISTOGRAM_BINS; i++)
        {
            distance_histogram_[i].r_min = r_min + i * bin_width;
            distance_histogram_[i].r_max = r_min + (i + 1) * bin_width;
            distance_histogram_[i].count = 0;
        }
        
        // Populate histogram
        for (const auto& sp : spherical_points_)
        {
            int bin_idx = static_cast<int>((sp.r - r_min) / bin_width);
            if (bin_idx >= NUM_HISTOGRAM_BINS)
                bin_idx = NUM_HISTOGRAM_BINS - 1;
            if (bin_idx < 0)
                bin_idx = 0;
            
            distance_histogram_[bin_idx].count++;
        }
        
        ROS_INFO("Distance histogram computed: r_min=%.2f, r_max=%.2f, bin_width=%.3f",
                 r_min, r_max, bin_width);
    }

    void LaserScannerNode::findDenseIntervals(int num_intervals)
    {
        if (distance_histogram_.empty())
        {
            ROS_WARN("Histogram not computed, cannot find dense intervals");
            return;
        }
        
        dense_intervals_.clear();
        
        int num_bins = static_cast<int>(distance_histogram_.size());
        
        // Get the full range
        float full_r_min = distance_histogram_[0].r_min;
        float full_r_max = distance_histogram_[num_bins - 1].r_max;
        
        // Step 1: Smooth the histogram to reduce noise (moving average with window=3)
        std::vector<float> smoothed(num_bins, 0.0f);
        for (int i = 0; i < num_bins; i++)
        {
            int count = 0;
            float sum = 0.0f;
            for (int j = std::max(0, i - 1); j <= std::min(num_bins - 1, i + 1); j++)
            {
                sum += distance_histogram_[j].count;
                count++;
            }
            smoothed[i] = sum / count;
        }
        
        // Step 2: Find local minima to identify valleys between peaks
        // These will be our split points
        std::vector<std::pair<int, float>> valleys; // (bin_index, smoothed_value)
        
        for (int i = 1; i < num_bins - 1; i++)
        {
            // A valley is where the smoothed value is lower than both neighbors
            if (smoothed[i] < smoothed[i - 1] && smoothed[i] < smoothed[i + 1])
            {
                valleys.push_back({i, smoothed[i]});
            }
        }
        
        // Step 3: Sort valleys by their depth (lowest smoothed value = deepest valley)
        std::sort(valleys.begin(), valleys.end(),
                  [](const auto& a, const auto& b) { return a.second < b.second; });
        
        // Step 4: Select the top (num_intervals - 1) deepest valleys as split points
        std::vector<int> split_indices;
        for (int i = 0; i < std::min(num_intervals - 1, static_cast<int>(valleys.size())); i++)
        {
            split_indices.push_back(valleys[i].first);
        }
        
        // Sort split indices by position
        std::sort(split_indices.begin(), split_indices.end());
        
        // Step 5: Create continuous intervals using split points
        // Add start and end boundaries
        std::vector<int> boundaries;
        boundaries.push_back(0);
        for (int idx : split_indices)
        {
            boundaries.push_back(idx);
        }
        boundaries.push_back(num_bins - 1);
        
        // Create intervals between boundaries (continuous, no gaps)
        for (size_t b = 0; b < boundaries.size() - 1; b++)
        {
            int start_idx = boundaries[b];
            int end_idx = boundaries[b + 1];
            
            // For the split point, the interval ends AT the split, next starts AT the split
            // This ensures continuity
            float interval_r_min, interval_r_max;
            
            if (b == 0)
            {
                interval_r_min = full_r_min;
            }
            else
            {
                // Start at the split point's r_min
                interval_r_min = distance_histogram_[start_idx].r_min;
            }
            
            if (b == boundaries.size() - 2)
            {
                interval_r_max = full_r_max;
            }
            else
            {
                // End at the next split point's r_min (so intervals are continuous)
                interval_r_max = distance_histogram_[end_idx].r_min;
            }
            
            // Calculate total points in this region
            int total_points = 0;
            for (int i = start_idx; i < end_idx; i++)
            {
                total_points += distance_histogram_[i].count;
            }
            // Include the last bin for the final interval
            if (b == boundaries.size() - 2)
            {
                total_points += distance_histogram_[end_idx].count;
            }
            
            DistanceInterval di;
            di.r_min = interval_r_min;
            di.r_max = interval_r_max;
            di.point_count = total_points;
            dense_intervals_.push_back(di);
        }
        
        ROS_INFO("Created %zu continuous intervals from %zu valleys", 
                 dense_intervals_.size(), valleys.size());
    }

    void LaserScannerNode::reportHistogram()
    {
        ROS_INFO("=== OBSERVATION PHASE COMPLETE - HISTOGRAM REPORT ===");
        ROS_INFO("Total spherical points collected (r >= %.1fm): %zu", 
                 HISTOGRAM_MIN_DISTANCE, spherical_points_.size());
        
        if (distance_histogram_.empty())
        {
            ROS_WARN("No histogram data available");
            return;
        }
        
        // Print histogram
        ROS_INFO("--- Distance Histogram (r in meters) ---");
        int max_count = 0;
        for (const auto& bin : distance_histogram_)
        {
            max_count = std::max(max_count, bin.count);
        }
        
        // Print non-empty bins
        for (const auto& bin : distance_histogram_)
        {
            if (bin.count > 0)
            {
                int bar_length = (bin.count * 40) / (max_count > 0 ? max_count : 1);
                std::string bar(bar_length, '#');
                ROS_INFO("[%.2f - %.2f]: %5d %s", bin.r_min, bin.r_max, bin.count, bar.c_str());
            }
        }
        
        // Print dense intervals
        ROS_INFO("--- Dense Distance Intervals ---");
        for (size_t i = 0; i < dense_intervals_.size(); i++)
        {
            const auto& di = dense_intervals_[i];
            ROS_INFO("Interval %zu: r = [%.2f - %.2f] m, points = %d",
                     i + 1, di.r_min, di.r_max, di.point_count);
        }
        
        ROS_INFO("=== END HISTOGRAM REPORT ===");
    }

    void LaserScannerNode::transitionToNormalMode()
    {
        // First, convert voxelized points to spherical coordinates
        convertVoxelsToSpherical();
        
        // Compute histogram from the spherical points
        computeDistanceHistogram();
        
        // Find dense intervals (e.g., 3 intervals)
        findDenseIntervals(3);
        
        // Report the histogram
        reportHistogram();
        
        observation_phase_active_ = false;
        
        // Start focused scanning phase
        startFocusedScanning();
    }

    std::vector<LaserScannerNode::SphericalPoint> LaserScannerNode::getPointsInInterval(float r_min, float r_max)
    {
        std::vector<SphericalPoint> filtered_points;
        
        for (const auto& sp : spherical_points_)
        {
            if (sp.r >= r_min && sp.r < r_max)
            {
                filtered_points.push_back(sp);
            }
        }
        
        return filtered_points;
    }

    void LaserScannerNode::clusterPointsIntoZones(const std::vector<SphericalPoint>& points, int interval_idx)
    {
        if (points.empty())
            return;
        
        // Use a simple grid-based clustering in pan-tilt space
        // Grid cell size based on typical scan pattern
        const float pan_cell_size = 0.15f;   // ~8.5 degrees
        const float tilt_cell_size = 0.10f;  // ~5.7 degrees
        
        // Find bounds
        float pan_min = std::numeric_limits<float>::max();
        float pan_max = std::numeric_limits<float>::lowest();
        float tilt_min = std::numeric_limits<float>::max();
        float tilt_max = std::numeric_limits<float>::lowest();
        
        for (const auto& p : points)
        {
            pan_min = std::min(pan_min, p.pan);
            pan_max = std::max(pan_max, p.pan);
            tilt_min = std::min(tilt_min, p.tilt);
            tilt_max = std::max(tilt_max, p.tilt);
        }
        
        // Create grid
        int pan_cells = static_cast<int>((pan_max - pan_min) / pan_cell_size) + 1;
        int tilt_cells = static_cast<int>((tilt_max - tilt_min) / tilt_cell_size) + 1;
        
        // Limit grid size
        pan_cells = std::min(pan_cells, 100);
        tilt_cells = std::min(tilt_cells, 100);
        
        if (pan_cells <= 0 || tilt_cells <= 0)
            return;
        
        // Grid to track which points belong to which cell
        std::vector<std::vector<std::vector<int>>> grid(pan_cells, 
            std::vector<std::vector<int>>(tilt_cells));
        
        // Assign points to grid cells
        for (size_t i = 0; i < points.size(); i++)
        {
            int pi = static_cast<int>((points[i].pan - pan_min) / pan_cell_size);
            int ti = static_cast<int>((points[i].tilt - tilt_min) / tilt_cell_size);
            pi = std::max(0, std::min(pi, pan_cells - 1));
            ti = std::max(0, std::min(ti, tilt_cells - 1));
            grid[pi][ti].push_back(i);
        }
        
        // Flood fill to find connected components
        std::vector<std::vector<int>> cell_label(pan_cells, std::vector<int>(tilt_cells, -1));
        int current_label = 0;
        
        for (int pi = 0; pi < pan_cells; pi++)
        {
            for (int ti = 0; ti < tilt_cells; ti++)
            {
                if (grid[pi][ti].empty() || cell_label[pi][ti] >= 0)
                    continue;
                
                // BFS flood fill
                std::vector<std::pair<int, int>> queue;
                queue.push_back({pi, ti});
                cell_label[pi][ti] = current_label;
                
                size_t head = 0;
                while (head < queue.size())
                {
                    int cp = queue[head].first;
                    int ct = queue[head].second;
                    head++;
                    
                    // Check 8 neighbors
                    for (int dp = -1; dp <= 1; dp++)
                    {
                        for (int dt = -1; dt <= 1; dt++)
                        {
                            if (dp == 0 && dt == 0) continue;
                            int np = cp + dp;
                            int nt = ct + dt;
                            if (np < 0 || np >= pan_cells || nt < 0 || nt >= tilt_cells)
                                continue;
                            if (grid[np][nt].empty() || cell_label[np][nt] >= 0)
                                continue;
                            cell_label[np][nt] = current_label;
                            queue.push_back({np, nt});
                        }
                    }
                }
                current_label++;
            }
        }
        
        // Create zones from clusters
        std::vector<std::vector<int>> cluster_points(current_label);
        for (int pi = 0; pi < pan_cells; pi++)
        {
            for (int ti = 0; ti < tilt_cells; ti++)
            {
                int label = cell_label[pi][ti];
                if (label >= 0)
                {
                    for (int idx : grid[pi][ti])
                    {
                        cluster_points[label].push_back(idx);
                    }
                }
            }
        }
        
        // Create ScanZone for each cluster with sufficient points
        // Use higher threshold to avoid zones with too few points
        const int min_cluster_points = 50;
        for (int label = 0; label < current_label; label++)
        {
            if (static_cast<int>(cluster_points[label].size()) < min_cluster_points)
                continue;
            
            ScanZone zone;
            zone.interval_idx = interval_idx;
            zone.point_count = cluster_points[label].size();
            
            // Calculate centroid and bounds
            float sum_pan = 0, sum_tilt = 0, sum_r = 0;
            float zone_pan_min = std::numeric_limits<float>::max();
            float zone_pan_max = std::numeric_limits<float>::lowest();
            float zone_tilt_min = std::numeric_limits<float>::max();
            float zone_tilt_max = std::numeric_limits<float>::lowest();
            
            for (int idx : cluster_points[label])
            {
                const auto& p = points[idx];
                sum_pan += p.pan;
                sum_tilt += p.tilt;
                sum_r += p.r;
                zone_pan_min = std::min(zone_pan_min, p.pan);
                zone_pan_max = std::max(zone_pan_max, p.pan);
                zone_tilt_min = std::min(zone_tilt_min, p.tilt);
                zone_tilt_max = std::max(zone_tilt_max, p.tilt);
            }
            
            zone.pan_center = sum_pan / cluster_points[label].size();
            zone.tilt_center = sum_tilt / cluster_points[label].size();
            zone.r_avg = sum_r / cluster_points[label].size();
            
            // Calculate center in cartesian
            zone.center_x = zone.r_avg * cos(zone.tilt_center) * cos(zone.pan_center);
            zone.center_y = zone.r_avg * cos(zone.tilt_center) * sin(zone.pan_center);
            zone.center_z = zone.r_avg * sin(zone.tilt_center);
            
            // Calculate deltas with margin
            float margin = 0.1f; // ~5.7 degrees margin
            zone.delta_pan = (zone_pan_max - zone_pan_min) / 2.0f + margin;
            zone.delta_tilt = (zone_tilt_max - zone_tilt_min) / 2.0f + margin;
            
            // Ensure minimum deltas
            zone.delta_pan = std::max(zone.delta_pan, 0.15f);   // minimum ~8.5 degrees
            zone.delta_tilt = std::max(zone.delta_tilt, 0.10f); // minimum ~5.7 degrees
            
            // Calculate scan parameters (check limits)
            calculateZoneScanParameters(zone);
            
            scan_zones_.push_back(zone);
        }
    }

    bool LaserScannerNode::computeJointAnglesForTarget(double x, double y, double z, double& pan_out, double& tilt_out)
    {
        // Use the same IK approach as focusPointCallback:
        // 1. Create a point in map frame
        // 2. Transform it to laser frame
        // 3. Compute the angular offset needed
        // 4. Apply to current joint angles to get target joint angles
        
        try
        {
            geometry_msgs::PointStamped pt_map;
            pt_map.header.frame_id = "map";
            pt_map.header.stamp = ros::Time(0);  // Use latest available transform
            pt_map.point.x = x;
            pt_map.point.y = y;
            pt_map.point.z = z;
            
            if (!tf_buffer_->canTransform("laser", "map", ros::Time(0), ros::Duration(0.5)))
            {
                ROS_WARN("Cannot transform to laser frame for IK");
                return false;
            }
            
            geometry_msgs::PointStamped pt_laser;
            tf_buffer_->transform(pt_map, pt_laser, "laser", ros::Duration(0.1));
            
            double lx = pt_laser.point.x;
            double ly = pt_laser.point.y;
            double lz = pt_laser.point.z;
            
            double range = sqrt(lx * lx + ly * ly + lz * lz);
            if (range < 0.01)
            {
                ROS_WARN("Target too close to laser origin for IK");
                return false;
            }
            
            // Same IK as focusPointCallback
            double current_pan = panangle_;
            double current_tilt = tiltangle_;
            double delta_pan = atan2(ly, lx);
            double delta_tilt = atan2(lz, lx);
            
            pan_out = current_pan - delta_pan;
            tilt_out = current_tilt + delta_tilt;
            
            // Handle target behind laser
            if (lx < 0.0)
            {
                tilt_out += PI;
            }
            
            // Normalize angles to [-π, π]
            while (pan_out > PI)
                pan_out -= 2.0 * PI;
            while (pan_out < -PI)
                pan_out += 2.0 * PI;
            while (tilt_out > PI)
                tilt_out -= 2.0 * PI;
            while (tilt_out < -PI)
                tilt_out += 2.0 * PI;
            
            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("TF exception in computeJointAnglesForTarget: %s", ex.what());
            return false;
        }
    }

    void LaserScannerNode::calculateZoneScanParameters(ScanZone& zone)
    {
        // Just ensure reasonable delta limits
        // The actual joint limit checking is done in startNextZoneScan using TF-based IK
        
        // Clamp tilt to physical limits (tilt is more straightforward)
        double tilt_max = zone.tilt_center + zone.delta_tilt;
        double tilt_min = zone.tilt_center - zone.delta_tilt;
        
        if (tilt_max > TILT_MAX)
        {
            float excess = tilt_max - TILT_MAX;
            zone.tilt_center -= excess / 2;
            zone.delta_tilt -= excess / 2;
        }
        if (tilt_min < TILT_MIN)
        {
            float deficit = TILT_MIN - tilt_min;
            zone.tilt_center += deficit / 2;
            zone.delta_tilt -= deficit / 2;
        }
        
        // Ensure positive deltas with reasonable minimum
        zone.delta_pan = std::max(zone.delta_pan, 0.10f);   // ~5.7 degrees min
        zone.delta_tilt = std::max(zone.delta_tilt, 0.10f); // ~5.7 degrees min
    }

    void LaserScannerNode::startFocusedScanning()
    {
        // Clear any existing zones
        scan_zones_.clear();
        
        ROS_INFO("=== STARTING FOCUSED SCANNING PHASE ===");
        
        // Process each interval and cluster points into zones
        for (size_t i = 0; i < dense_intervals_.size(); i++)
        {
            const auto& interval = dense_intervals_[i];
            ROS_INFO("Processing interval %zu: r = [%.2f - %.2f] m", 
                     i + 1, interval.r_min, interval.r_max);
            
            auto points = getPointsInInterval(interval.r_min, interval.r_max);
            ROS_INFO("  Found %zu points in interval", points.size());
            
            clusterPointsIntoZones(points, i);
        }
        
        ROS_INFO("Total scan zones identified: %zu", scan_zones_.size());
        
        // Report zones
        for (size_t i = 0; i < scan_zones_.size(); i++)
        {
            const auto& zone = scan_zones_[i];
            ROS_INFO("Zone %zu: Interval %d, Center (pan=%.1f°, tilt=%.1f°), "
                     "Deltas (±%.1f°, ±%.1f°), r_avg=%.2fm, points=%d",
                     i + 1, zone.interval_idx + 1,
                     zone.pan_center * 180.0 / PI, zone.tilt_center * 180.0 / PI,
                     zone.delta_pan * 180.0 / PI, zone.delta_tilt * 180.0 / PI,
                     zone.r_avg, zone.point_count);
        }
        
        if (scan_zones_.empty())
        {
            ROS_WARN("No zones found for focused scanning, transitioning to manual mode");
            transitionToManualMode();
            return;
        }
        
        // Start focused scanning
        focused_scanning_active_ = true;
        current_zone_idx_ = 0;
        
        // Start first zone scan
        startNextZoneScan();
    }

    bool LaserScannerNode::startNextZoneScan()
    {
        if (current_zone_idx_ >= static_cast<int>(scan_zones_.size()))
        {
            ROS_INFO("=== ALL ZONES SCANNED ===");
            focused_scanning_active_ = false;
            clearZoneVisualization();
            transitionToManualMode();
            return false;
        }
        
        const auto& zone = scan_zones_[current_zone_idx_];
        
        // Get the interval bounds for visualization
        float r_min = 0.0f, r_max = 0.0f;
        if (zone.interval_idx >= 0 && zone.interval_idx < static_cast<int>(dense_intervals_.size()))
        {
            r_min = dense_intervals_[zone.interval_idx].r_min;
            r_max = dense_intervals_[zone.interval_idx].r_max;
        }
        else
        {
            // Fallback: use zone's r_avg with some margin
            r_min = zone.r_avg * 0.8f;
            r_max = zone.r_avg * 1.2f;
            ROS_WARN("Zone interval index %d out of range, using fallback r_min=%.2f, r_max=%.2f",
                     zone.interval_idx, r_min, r_max);
        }
        
        ROS_INFO("=== STARTING ZONE %d/%zu SCAN ===", 
                 current_zone_idx_ + 1, scan_zones_.size());
        ROS_INFO("Zone center (map frame): x=%.2f, y=%.2f, z=%.2f",
                 zone.center_x, zone.center_y, zone.center_z);
        ROS_INFO("Zone spherical: Pan=%.1f°, Tilt=%.1f°, r=%.2fm",
                 zone.pan_center * 180.0 / PI, zone.tilt_center * 180.0 / PI, zone.r_avg);
        ROS_INFO("Zone deltas: Pan=±%.1f°, Tilt=±%.1f°",
                 zone.delta_pan * 180.0 / PI, zone.delta_tilt * 180.0 / PI);
        ROS_INFO("Visualizing volume: r=[%.2f, %.2f]m", r_min, r_max);
        
        // Publish zone visualization (uses map frame azimuth angles)
        publishZoneVisualization(zone, r_min, r_max);
        
        // Use TF-based IK to compute joint angles for the zone center
        // This uses the same approach as focusPointCallback for consistency
        double pan_joint_center, tilt_joint_center;
        if (!computeJointAnglesForTarget(zone.center_x, zone.center_y, zone.center_z,
                                          pan_joint_center, tilt_joint_center))
        {
            ROS_WARN("Failed to compute IK for zone center, skipping zone");
            current_zone_idx_++;
            return startNextZoneScan();
        }
        
        ROS_INFO("IK solution: Pan joint=%.1f°, Tilt joint=%.1f°",
                 pan_joint_center * 180.0 / PI, tilt_joint_center * 180.0 / PI);
        
        // Check if within physical limits
        if (pan_joint_center > PAN_MAX || pan_joint_center < PAN_MIN)
        {
            ROS_WARN("Pan joint %.1f° outside limits [%.1f°, %.1f°], skipping zone",
                     pan_joint_center * 180.0 / PI, PAN_MIN * 180.0 / PI, PAN_MAX * 180.0 / PI);
            current_zone_idx_++;
            return startNextZoneScan();
        }
        
        double v_target;
        pnh_.param("target_velocity", v_target, 1.0);
        scanner_ = std::make_unique<ParametricScanner>(v_target, zone.delta_pan, zone.delta_tilt);
        scanner_->setOffsets(pan_joint_center, tilt_joint_center);
        scanner_->reset();
        
        // Update current pattern limits (in joint space)
        current_pan_limit_max_ = pan_joint_center + zone.delta_pan;
        current_pan_limit_min_ = pan_joint_center - zone.delta_pan;
        current_tilt_limit_max_ = tilt_joint_center + zone.delta_tilt;
        current_tilt_limit_min_ = tilt_joint_center - zone.delta_tilt;
        
        // Place debug marker at zone center
        placeDebugPoint(zone.center_x, zone.center_y, zone.center_z, 0, 255, 0);
        
        // Start zone scan timer
        zone_scan_start_time_ = std::chrono::high_resolution_clock::now();
        
        return true;
    }

    void LaserScannerNode::processZoneScanning()
    {
        if (!focused_scanning_active_)
            return;
        
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = current_time - zone_scan_start_time_;
        
        if (elapsed.count() >= ZONE_SCAN_DURATION)
        {
            ROS_INFO("Zone %d/%zu scan complete after %.2f seconds",
                     current_zone_idx_ + 1, scan_zones_.size(), elapsed.count());
            
            current_zone_idx_++;
            startNextZoneScan();
        }
    }

    void LaserScannerNode::transitionToManualMode()
    {
        // Get normal operation limits from parameters
        double delta_1_deg, delta_2_deg;
        pnh_.param("pan_limit_deg", delta_1_deg, 30.0);
        pnh_.param("tilt_limit_deg", delta_2_deg, 20.0);
        
        double delta_1 = delta_1_deg * PI / 180.0;
        double delta_2 = delta_2_deg * PI / 180.0;
        
        // Update scanner to use normal operation limits
        double v_target;
        pnh_.param("target_velocity", v_target, 1.0);
        scanner_ = std::make_unique<ParametricScanner>(v_target, delta_1, delta_2);
        scanner_->reset();
        
        // Update current pattern limits
        current_pan_limit_max_ = delta_1;
        current_pan_limit_min_ = -delta_1;
        current_tilt_limit_max_ = delta_2;
        current_tilt_limit_min_ = -delta_2;
        
        focused_scanning_active_ = false;
        
        // Clear any zone visualization
        clearZoneVisualization();
        
        ROS_INFO("=== TRANSITIONED TO MANUAL OPERATION MODE ===");
        ROS_INFO("Now accepting point markers for focused scanning");
        ROS_INFO("Pan limit: ±%.1f degrees, Tilt limit: ±%.1f degrees", delta_1_deg, delta_2_deg);
    }

    void LaserScannerNode::publishZoneVisualization(const ScanZone& zone, float r_min, float r_max)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "zone_volume";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        
        // Semi-transparent cyan color
        marker.color.r = 0.0;
        marker.color.g = 0.8;
        marker.color.b = 1.0;
        marker.color.a = 0.3;
        
        // Angular bounds
        float pan_min = zone.pan_center - zone.delta_pan;
        float pan_max = zone.pan_center + zone.delta_pan;
        float tilt_min = zone.tilt_center - zone.delta_tilt;
        float tilt_max = zone.tilt_center + zone.delta_tilt;
        
        // Resolution for mesh generation
        const int pan_steps = 16;
        const int tilt_steps = 8;
        const int r_steps = 2;  // Just inner and outer surfaces
        
        // Helper lambda to convert spherical to cartesian
        auto spherical_to_xyz = [](float r, float pan, float tilt) -> geometry_msgs::Point {
            geometry_msgs::Point p;
            p.x = r * cos(tilt) * cos(pan);
            p.y = r * cos(tilt) * sin(pan);
            p.z = r * sin(tilt);
            return p;
        };
        
        // Build mesh triangles
        float d_pan = (pan_max - pan_min) / pan_steps;
        float d_tilt = (tilt_max - tilt_min) / tilt_steps;
        
        // 1. Outer spherical surface (at r_max)
        for (int i = 0; i < pan_steps; i++)
        {
            for (int j = 0; j < tilt_steps; j++)
            {
                float p0 = pan_min + i * d_pan;
                float p1 = pan_min + (i + 1) * d_pan;
                float t0 = tilt_min + j * d_tilt;
                float t1 = tilt_min + (j + 1) * d_tilt;
                
                // Two triangles per quad
                geometry_msgs::Point v00 = spherical_to_xyz(r_max, p0, t0);
                geometry_msgs::Point v10 = spherical_to_xyz(r_max, p1, t0);
                geometry_msgs::Point v01 = spherical_to_xyz(r_max, p0, t1);
                geometry_msgs::Point v11 = spherical_to_xyz(r_max, p1, t1);
                
                marker.points.push_back(v00);
                marker.points.push_back(v10);
                marker.points.push_back(v11);
                
                marker.points.push_back(v00);
                marker.points.push_back(v11);
                marker.points.push_back(v01);
            }
        }
        
        // 2. Inner spherical surface (at r_min)
        for (int i = 0; i < pan_steps; i++)
        {
            for (int j = 0; j < tilt_steps; j++)
            {
                float p0 = pan_min + i * d_pan;
                float p1 = pan_min + (i + 1) * d_pan;
                float t0 = tilt_min + j * d_tilt;
                float t1 = tilt_min + (j + 1) * d_tilt;
                
                geometry_msgs::Point v00 = spherical_to_xyz(r_min, p0, t0);
                geometry_msgs::Point v10 = spherical_to_xyz(r_min, p1, t0);
                geometry_msgs::Point v01 = spherical_to_xyz(r_min, p0, t1);
                geometry_msgs::Point v11 = spherical_to_xyz(r_min, p1, t1);
                
                // Reverse winding for inner surface
                marker.points.push_back(v00);
                marker.points.push_back(v11);
                marker.points.push_back(v10);
                
                marker.points.push_back(v00);
                marker.points.push_back(v01);
                marker.points.push_back(v11);
            }
        }
        
        // 3. Side faces (constant pan edges)
        for (int j = 0; j < tilt_steps; j++)
        {
            float t0 = tilt_min + j * d_tilt;
            float t1 = tilt_min + (j + 1) * d_tilt;
            
            // Left side (pan_min)
            geometry_msgs::Point li0 = spherical_to_xyz(r_min, pan_min, t0);
            geometry_msgs::Point li1 = spherical_to_xyz(r_min, pan_min, t1);
            geometry_msgs::Point lo0 = spherical_to_xyz(r_max, pan_min, t0);
            geometry_msgs::Point lo1 = spherical_to_xyz(r_max, pan_min, t1);
            
            marker.points.push_back(li0);
            marker.points.push_back(lo0);
            marker.points.push_back(lo1);
            marker.points.push_back(li0);
            marker.points.push_back(lo1);
            marker.points.push_back(li1);
            
            // Right side (pan_max)
            geometry_msgs::Point ri0 = spherical_to_xyz(r_min, pan_max, t0);
            geometry_msgs::Point ri1 = spherical_to_xyz(r_min, pan_max, t1);
            geometry_msgs::Point ro0 = spherical_to_xyz(r_max, pan_max, t0);
            geometry_msgs::Point ro1 = spherical_to_xyz(r_max, pan_max, t1);
            
            marker.points.push_back(ri0);
            marker.points.push_back(ro1);
            marker.points.push_back(ro0);
            marker.points.push_back(ri0);
            marker.points.push_back(ri1);
            marker.points.push_back(ro1);
        }
        
        // 4. Top and bottom faces (constant tilt edges)
        for (int i = 0; i < pan_steps; i++)
        {
            float p0 = pan_min + i * d_pan;
            float p1 = pan_min + (i + 1) * d_pan;
            
            // Bottom (tilt_min)
            geometry_msgs::Point bi0 = spherical_to_xyz(r_min, p0, tilt_min);
            geometry_msgs::Point bi1 = spherical_to_xyz(r_min, p1, tilt_min);
            geometry_msgs::Point bo0 = spherical_to_xyz(r_max, p0, tilt_min);
            geometry_msgs::Point bo1 = spherical_to_xyz(r_max, p1, tilt_min);
            
            marker.points.push_back(bi0);
            marker.points.push_back(bo1);
            marker.points.push_back(bo0);
            marker.points.push_back(bi0);
            marker.points.push_back(bi1);
            marker.points.push_back(bo1);
            
            // Top (tilt_max)
            geometry_msgs::Point ti0 = spherical_to_xyz(r_min, p0, tilt_max);
            geometry_msgs::Point ti1 = spherical_to_xyz(r_min, p1, tilt_max);
            geometry_msgs::Point to0 = spherical_to_xyz(r_max, p0, tilt_max);
            geometry_msgs::Point to1 = spherical_to_xyz(r_max, p1, tilt_max);
            
            marker.points.push_back(ti0);
            marker.points.push_back(to0);
            marker.points.push_back(to1);
            marker.points.push_back(ti0);
            marker.points.push_back(to1);
            marker.points.push_back(ti1);
        }
        
        marker_pub_.publish(marker);
        
        // Also publish edge wireframe for better visibility
        visualization_msgs::Marker wireframe;
        wireframe.header.frame_id = "map";
        wireframe.header.stamp = ros::Time::now();
        wireframe.ns = "zone_wireframe";
        wireframe.id = 0;
        wireframe.type = visualization_msgs::Marker::LINE_LIST;
        wireframe.action = visualization_msgs::Marker::ADD;
        wireframe.pose.orientation.w = 1.0;
        wireframe.scale.x = 0.03;  // Line width
        wireframe.color.r = 0.0;
        wireframe.color.g = 1.0;
        wireframe.color.b = 0.5;
        wireframe.color.a = 1.0;
        
        // Draw main corner edges
        std::vector<std::pair<float, float>> corners = {
            {pan_min, tilt_min}, {pan_max, tilt_min},
            {pan_min, tilt_max}, {pan_max, tilt_max}
        };
        
        // Radial edges at corners
        for (size_t ci = 0; ci < corners.size(); ci++)
        {
            float pan = corners[ci].first;
            float tilt = corners[ci].second;
            geometry_msgs::Point inner = spherical_to_xyz(r_min, pan, tilt);
            geometry_msgs::Point outer = spherical_to_xyz(r_max, pan, tilt);
            wireframe.points.push_back(inner);
            wireframe.points.push_back(outer);
        }
        
        // Arc edges on inner and outer surfaces
        for (float r : {r_min, r_max})
        {
            // Pan arcs at tilt_min and tilt_max
            for (float tilt : {tilt_min, tilt_max})
            {
                for (int i = 0; i < pan_steps; i++)
                {
                    float p0 = pan_min + i * d_pan;
                    float p1 = pan_min + (i + 1) * d_pan;
                    wireframe.points.push_back(spherical_to_xyz(r, p0, tilt));
                    wireframe.points.push_back(spherical_to_xyz(r, p1, tilt));
                }
            }
            
            // Tilt arcs at pan_min and pan_max
            for (float pan : {pan_min, pan_max})
            {
                for (int j = 0; j < tilt_steps; j++)
                {
                    float t0 = tilt_min + j * d_tilt;
                    float t1 = tilt_min + (j + 1) * d_tilt;
                    wireframe.points.push_back(spherical_to_xyz(r, pan, t0));
                    wireframe.points.push_back(spherical_to_xyz(r, pan, t1));
                }
            }
        }
        
        marker_pub_.publish(wireframe);
    }

    void LaserScannerNode::clearZoneVisualization()
    {
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = "zone_volume";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::Marker::DELETE;
        marker_pub_.publish(clear_marker);
        
        clear_marker.ns = "zone_wireframe";
        marker_pub_.publish(clear_marker);
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

        // Reset the scanner to ensure it starts from a clean state
        // This sets t_param to a value where tilt is near zero
        scanner_->reset();
        
        // Get the initial target position
        double initial_pan, initial_tilt;
        scanner_->getTargetAngles(initial_pan, initial_tilt);
        
        ROS_INFO("Moving to initial scan position: Pan=%.1f°, Tilt=%.1f°", 
                 initial_pan * 180.0 / PI, initial_tilt * 180.0 / PI);
        
        // Wait for robot to reach initial position before starting observation timer
        const double position_threshold = 0.05; // ~3 degrees tolerance
        int stable_count = 0;
        const int required_stable_frames = 10; // Must be stable for 10 frames
        
        while (ros::ok() && stable_count < required_stable_frames)
        {
            // Publish the initial target position
            publishJointCommand(initial_tilt, initial_pan);
            
            // Check if robot is near target
            double pan_error = std::abs(panangle_ - initial_pan);
            double tilt_error = std::abs(tiltangle_ - initial_tilt);
            
            if (pan_error < position_threshold && tilt_error < position_threshold)
            {
                stable_count++;
            }
            else
            {
                stable_count = 0;
            }
            
            ros::spinOnce();
            loop.sleep();
        }
        
        ROS_INFO("Robot reached initial position. Starting observation phase...");

        // Now start scanning in observation mode
        scanning_active_ = true;
        observation_phase_active_ = true;
        scan_start_time_ = std::chrono::high_resolution_clock::now();
        
        // Reset again to sync the timing
        scanner_->reset();
        
        ROS_INFO("=== OBSERVATION PHASE STARTED (%.1f seconds) ===", OBSERVATION_DURATION);
        ROS_INFO("Using observation deltas: Pan=±%.1f°, Tilt=±%.1f°", 
                 delta_1_observe_ * 180.0 / PI, delta_2_observe_ * 180.0 / PI);

        while (ros::ok())
        {
            double pan_target, tilt_target;

            // Check for observation phase completion
            if (observation_phase_active_)
            {
                auto current_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = current_time - scan_start_time_;
                
                if (elapsed.count() >= OBSERVATION_DURATION)
                {
                    ROS_INFO("Observation phase complete after %.2f seconds", elapsed.count());
                    transitionToNormalMode();
                }
            }
            
            // Check for focused scanning phase (zone scanning)
            if (focused_scanning_active_)
            {
                processZoneScanning();
            }

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
