// Real system laser scanning with parametric position control
// Uses improved position-based control matching vel_publisher.cpp pattern
// Compatible with Arduino QuickPID position controller
// Uses TF-based voxelization from laser_to_pointcloud.cpp

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/JointState.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <fstream>
#include <std_msgs/Bool.h>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <unordered_map>

#define PI 3.14159265359

using namespace std;

// ========================================
// Voxel Structures (from laser_to_pointcloud.cpp)
// ========================================

// Voxel structure for global voxelization with arithmetic averaging
struct VoxelPoint
{
    float x, y, z;         // Current averaged position
    float sensor_distance; // Distance to sensor when point was acquired (for reference)
    int count;             // Number of points accumulated

    VoxelPoint() : x(0), y(0), z(0), sensor_distance(0), count(0) {}
    VoxelPoint(float x_, float y_, float z_, float dist)
        : x(x_), y(y_), z(z_), sensor_distance(dist), count(1) {}

    // Update position using simple arithmetic averaging
    void updatePosition(float new_x, float new_y, float new_z, float new_distance)
    {
        float new_count = count + 1;
        x = (x * count + new_x) / new_count;
        y = (y * count + new_y) / new_count;
        z = (z * count + new_z) / new_count;
        count++;

        if (new_distance < sensor_distance)
        {
            sensor_distance = new_distance;
        }
    }
};

// Voxel key for hash map
struct VoxelKey
{
    int x, y, z;

    VoxelKey(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}

    bool operator==(const VoxelKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Hash function for VoxelKey
struct VoxelKeyHash
{
    size_t operator()(const VoxelKey &key) const
    {
        return hash<int>()(key.x) ^ (hash<int>()(key.y) << 1) ^ (hash<int>()(key.z) << 2);
    }
};

// ========================================
// Global Variables
// ========================================

// Voxel configuration
const float GLOBAL_VOXEL_SIZE = 0.05f; // 50mm consistent global voxel size
unordered_map<VoxelKey, VoxelPoint, VoxelKeyHash> voxel_grid;

// Current joint positions from Arduino
double tiltangle = 0.0;
double panangle = 0.0;

// Target joint positions (from parametric scanning)
double tiltangle_goal = 0.0;
double panangle_goal = 0.0;

// Angular velocity tracking for path-following speed
double last_tiltangle_for_velocity = 0.0;
double last_panangle_for_velocity = 0.0;
auto last_velocity_time = std::chrono::high_resolution_clock::now();

// Path-following speed tracking (speed along parametric curve)
double path_speed_sum = 0.0;
int path_speed_sample_count = 0;

// Parametric scanning variables (matching vel_publisher.cpp)
double t_param = 0.0;                      // Parameter time for scanning equations
double v_target = 1.0;                     // Target velocity in rad/s (slower for real system)
double delta_1 = 60.0 * PI / 180.0;        // Pan limit (60 degrees)
double delta_2 = 45.0 * PI / 180.0;        // Tilt limit (45 degrees)
double sqrt2_over_100 = sqrt(2.0) / 100.0; // Irrational frequency component
auto last_param_update_time = std::chrono::high_resolution_clock::now();

// Laser scan data
std::vector<float> laser_scan;
sensor_msgs::LaserScan last_laser;
double laser_increment = 0.0;

// === De-skew parameters ===
double time_offset_s = 0.0; // Δt between LaserScan stamps and TF/joint stamps (seconds)
bool deskew_per_ray = true; // allow turning off for debugging

// Pointcloud variables
sensor_msgs::PointCloud2 cloud_out;
int ctn = 0; // Counter for processed scans
int scanSize;

// TF2 transform handling
std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;

// Scanning control
bool scanning_active = false;
bool save_pointcloud = false;
bool joint_states_received = false; // Track if we've received joint states
double scan_duration = 0.0;         // Duration of scanning in seconds
auto scan_start_time = std::chrono::high_resolution_clock::now();

// ========================================
// Voxelization Helper Functions
// ========================================

VoxelKey getVoxelKey(float x, float y, float z, float voxel_size)
{
    int vx = static_cast<int>(floor(x / voxel_size));
    int vy = static_cast<int>(floor(y / voxel_size));
    int vz = static_cast<int>(floor(z / voxel_size));
    return VoxelKey(vx, vy, vz);
}

void insertPointIntoVoxelGrid(float x, float y, float z, float distance)
{
    VoxelKey key = getVoxelKey(x, y, z, GLOBAL_VOXEL_SIZE);

    auto it = voxel_grid.find(key);
    if (it != voxel_grid.end())
    {
        it->second.updatePosition(x, y, z, distance);
    }
    else
    {
        voxel_grid[key] = VoxelPoint(x, y, z, distance);
    }
}

vector<VoxelPoint> getPointsFromVoxelGrid()
{
    vector<VoxelPoint> points;
    points.reserve(voxel_grid.size());

    for (const auto &pair : voxel_grid)
    {
        points.push_back(pair.second);
    }

    return points;
}

void clearVoxelGrid()
{
    voxel_grid.clear();
}

// ========================================
// Parametric Scanning Functions (from vel_publisher.cpp)
// ========================================

double f1(double t)
{
    // Pan function: sinusoidal motion within ±delta_1 limits
    return delta_1 * sin(t);
}

double f2(double t)
{
    // Tilt function: non-repetitive scanning pattern
    double freq = 3.0 + sqrt2_over_100;
    return (PI / 2.0) * (cos(freq * t) + 1.0) / 2.0 - PI / 4.0;
}

// Derivative functions for velocity calculation
double df1_dt(double t, double dt)
{
    return (f1(t + dt) - f1(t)) / dt;
}

double df2_dt(double t, double dt)
{
    return (f2(t + dt) - f2(t)) / dt;
}

// Calculate instantaneous speed for constant velocity implementation
double calculate_speed(double t, double dt)
{
    double df1 = df1_dt(t, dt);
    double df2 = df2_dt(t, dt);
    return sqrt(df1 * df1 + df2 * df2);
}

// Update parameter time for constant velocity scanning
void update_parameter_time()
{
    // Calculate actual elapsed time since last update
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dt_elapsed = current_time - last_param_update_time;
    double delta_t_actual = dt_elapsed.count();

    // Always update the timestamp to prevent jumps on next call
    last_param_update_time = current_time;

    if (delta_t_actual < 0.001)
    {
        // Too soon, skip this update to avoid numerical issues
        return;
    }

    double dt_small = 0.001;
    double current_speed = calculate_speed(t_param, dt_small);

    if (current_speed > 0.0)
    {
        // Advance along the curve at v_target speed using actual elapsed time
        double delta_s = v_target * delta_t_actual;
        double delta_t_param = delta_s / current_speed;
        t_param += delta_t_param;
    }
    else
    {
        t_param += delta_t_actual;
    }
}

// Get target angles using parametric scanning
void get_parametric_targets(double &pan_target, double &tilt_target)
{
    pan_target = f1(t_param);
    tilt_target = f2(t_param);

    // Optional: clamp to physical limits
    // pan_target = std::max(-delta_1, std::min(delta_1, pan_target));
    // tilt_target = std::max(-delta_2, std::min(delta_2, tilt_target));
}

// ========================================
// ROS Callbacks
// ========================================

// Laser scan callback - Process each scan immediately with current joint states
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_increment = msg->angle_increment;
    laser_scan = msg->ranges;
    last_laser = *msg;

    // Process laser scan into voxel grid immediately if scanning is active
    if (scanning_active && !laser_scan.empty() && tf_buffer && joint_states_received)
    {
        scanSize = static_cast<int>(laser_scan.size());

        // Use message-provided angles/timings
        const float angle_min = msg->angle_min;
        const float angle_inc = msg->angle_increment;
        double time_inc = msg->time_increment;
        if (time_inc <= 0.0 && scanSize > 1)
        {
            // conservative fallback
            ROS_WARN_THROTTLE(5, "LaserScan time_increment not provided, using estimated value.");
            time_inc = msg->scan_time / static_cast<double>(scanSize);
        }
        for (int i = 0; i < scanSize; i++)
        {
            const float r = laser_scan[i];
            if (!std::isfinite(r) && !std::isnan(r))
                continue;
            // Skip invalid measurements (beyond 4m range and within 10cm)
            if (r <= 0.10 || r > 4.0)
                continue;

            // Convert laser point to 2D coordinates in laser frame
            float angle = angle_inc * i + angle_min;

            // --- Per-beam timestamp (deskew) ---
            ros::Time beam_stamp = msg->header.stamp;
            if (deskew_per_ray)
            {
                const double dt = static_cast<double>(i) * time_inc + time_offset_s;
                beam_stamp = msg->header.stamp + ros::Duration(dt);
            }

            // Point in laser frame
            geometry_msgs::PointStamped pt_laser;
            pt_laser.header.frame_id = "laser_cal";
            pt_laser.header.stamp = beam_stamp; // << per-beam time here
            pt_laser.point.x = r * std::cos(angle);
            pt_laser.point.y = r * std::sin(angle);
            pt_laser.point.z = 0.0;

            // Quick transform guard to avoid exceptions
            if (!tf_buffer->canTransform("map", "laser_cal", beam_stamp, ros::Duration(0.02)))
            {
                continue; // no transform for this beam time
            }
            try
            {
                geometry_msgs::PointStamped pt_map;
                tf_buffer->transform(pt_laser, pt_map, "map", ros::Duration(0.02));
                const float x = pt_map.point.x;
                const float y = pt_map.point.y;
                const float z = pt_map.point.z;

                // Insert into voxel grid
                insertPointIntoVoxelGrid(x, y, z, r);
            }
            catch (tf2::TransformException &ex)
            {
                // Skip this point if transform fails
                continue;
            }
        }

        ctn++;

        // Progress reporting every 50 scans
        if (ctn % 50 == 0)
        {
            float avg_points_per_voxel = 0.0f;
            if (!voxel_grid.empty())
            {
                float total_points = 0.0f;
                for (const auto &pair : voxel_grid)
                {
                    total_points += pair.second.count;
                }
                avg_points_per_voxel = total_points / voxel_grid.size();
            }

            // Calculate average path-following speed for this interval
            double avg_path_speed = 0.0;
            if (path_speed_sample_count > 0)
            {
                avg_path_speed = path_speed_sum / path_speed_sample_count;
            }

            ROS_INFO("Processed %d scans | Voxels: %lu (avg %.1f pts/voxel) | Path speed: %.3f rad/s (target: %.2f)",
                     ctn, voxel_grid.size(), avg_points_per_voxel, avg_path_speed, v_target);

            // Reset accumulators for next interval
            path_speed_sum = 0.0;
            path_speed_sample_count = 0;
        }
    }
}

// Joint state callback from Arduino - Track joint positions and velocities
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // Arduino publishes: position[0] = tilt (radians), position[1] = pan (radians)
    // ROS standard: JointState positions are in radians
    if (msg->position.size() >= 2)
    {
        tiltangle = msg->position[0]; // Tilt angle in radians
        panangle = msg->position[1];  // Pan angle in radians
        joint_states_received = true; // Mark that we've received at least one message

        // Calculate angular velocities
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dt = current_time - last_velocity_time;

        if (dt.count() > 0.001)
        { // Minimum 1ms between samples to avoid division by zero
            double tilt_velocity = (tiltangle - last_tiltangle_for_velocity) / dt.count();
            double pan_velocity = (panangle - last_panangle_for_velocity) / dt.count();

            // Calculate path-following speed: sqrt(v_tilt^2 + v_pan^2)
            // This is the actual speed along the 2D parametric curve in joint space
            double path_speed = sqrt(tilt_velocity * tilt_velocity + pan_velocity * pan_velocity);

            // Accumulate for averaging
            path_speed_sum += path_speed;
            path_speed_sample_count++;

            // Update last values
            last_tiltangle_for_velocity = tiltangle;
            last_panangle_for_velocity = panangle;
            last_velocity_time = current_time;
        }
    }

    // Check if scan duration has elapsed
    if (scanning_active)
    {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = current_time - scan_start_time;

        if (elapsed.count() >= scan_duration && scan_duration > 0.0)
        {
            scanning_active = false;
            save_pointcloud = true;
            ROS_INFO("Scan complete after %.2f seconds", elapsed.count());
        }
    }
}

// ========================================
// Main Function
// ========================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_real_position");
    ros::NodeHandle nh;

    // Initialize TF2 buffer and listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>(ros::Duration(30.0));
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

    // Get scanning parameters from ROS parameters
    nh.param("target_velocity", v_target, 1.0);
    nh.param("pan_limit_deg", delta_1, 60.0);
    nh.param("tilt_limit_deg", delta_2, 45.0);
    nh.param("scan_duration", scan_duration, 0.0);
    nh.param("time_offset_s", time_offset_s, 0.0);
    nh.param("deskew_per_ray", deskew_per_ray, true);

    // Convert degrees to radians
    delta_1 = delta_1 * PI / 180.0;
    delta_2 = delta_2 * PI / 180.0;

    ROS_INFO("=== Parametric Position-Based Laser Scanner with TF Voxelization ===");
    ROS_INFO("Target velocity: %.2f rad/s", v_target);
    ROS_INFO("Pan limit: ±%.1f degrees", delta_1 * 180.0 / PI);
    ROS_INFO("Tilt limit: ±%.1f degrees", delta_2 * 180.0 / PI);
    ROS_INFO("Scan duration: %.1f seconds", scan_duration);
    ROS_INFO("Using TF transforms instead of hardcoded kinematics");
    ROS_INFO("Voxel size: %.0fmm", GLOBAL_VOXEL_SIZE * 1000.0f);

    // Subscribers
    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 100, jointStateCallback);
    ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laserCallback);

    // Publishers
    ros::Publisher joint_cmd_pub = nh.advertise<sensor_msgs::JointState>("/joint_command", 10);
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("output", 10);

    sensor_msgs::JointState joint_cmd;
    joint_cmd.name.push_back("joint1"); // Tilt
    joint_cmd.name.push_back("joint2"); // Pan
    joint_cmd.position.resize(2);

    ros::Rate loop(50); // 50Hz control loop

    // Wait for initial joint state
    ROS_INFO("Waiting for joint states from Arduino...");
    while (ros::ok() && !joint_states_received)
    {
        ros::spinOnce();
        loop.sleep();
    }
    ROS_INFO("Joint states received. Waiting for TF transforms...");
    ros::Duration(2.0).sleep();

    // Start scanning
    scanning_active = true;
    scan_start_time = std::chrono::high_resolution_clock::now();
    last_param_update_time = std::chrono::high_resolution_clock::now(); // Initialize for pattern updates
    ROS_INFO("=== SCANNING STARTED ===");

    while (ros::ok())
    {
        // Update parametric pattern at loop rate (50Hz) for smooth commands
        if (scanning_active)
        {
            update_parameter_time();
            get_parametric_targets(panangle_goal, tiltangle_goal);
        }

        // Publish position commands if scanning is active
        if (scanning_active)
        {
            joint_cmd.header.stamp = ros::Time::now();
            joint_cmd.position[0] = tiltangle_goal; // Tilt target (radians)
            joint_cmd.position[1] = panangle_goal;  // Pan target (radians)
            joint_cmd_pub.publish(joint_cmd);
        }
        else
        {
            // Return to home position when not scanning
            joint_cmd.header.stamp = ros::Time::now();
            joint_cmd.position[0] = 0.0; // Tilt home (radians)
            joint_cmd.position[1] = 0.0; // Pan home (radians)
            joint_cmd_pub.publish(joint_cmd);
        }

        // Generate pointcloud from voxel grid
        vector<VoxelPoint> voxel_points = getPointsFromVoxelGrid();

        if (!voxel_points.empty())
        {
            // Setup point cloud message
            cloud_out.header.frame_id = "map";
            cloud_out.header.stamp = ros::Time::now();
            cloud_out.height = 1;
            cloud_out.width = voxel_points.size();
            cloud_out.fields.resize(3);

            cloud_out.fields[0].name = "x";
            cloud_out.fields[0].offset = 0;
            cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            cloud_out.fields[0].count = 1;

            cloud_out.fields[1].name = "y";
            cloud_out.fields[1].offset = 4;
            cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            cloud_out.fields[1].count = 1;

            cloud_out.fields[2].name = "z";
            cloud_out.fields[2].offset = 8;
            cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            cloud_out.fields[2].count = 1;

            cloud_out.point_step = 12;
            cloud_out.row_step = cloud_out.point_step * cloud_out.width;
            cloud_out.data.resize(cloud_out.row_step * cloud_out.height);
            cloud_out.is_dense = false;

            // Fill point cloud data from voxel grid
            for (size_t i = 0; i < voxel_points.size(); ++i)
            {
                float *pstep = (float *)&cloud_out.data[i * cloud_out.point_step];
                pstep[0] = voxel_points[i].x;
                pstep[1] = voxel_points[i].y;
                pstep[2] = voxel_points[i].z;
            }
        }

        // Publish pointcloud
        pcl_pub.publish(cloud_out);

        // Save pointcloud if requested
        if (save_pointcloud)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(cloud_out, *pcl_cloud);

            // Generate filename with timestamp
            auto now = std::chrono::system_clock::now();
            auto timestamp = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << "pointcloud_" << timestamp << ".pcd";

            pcl::io::savePCDFileASCII(ss.str(), *pcl_cloud);
            ROS_INFO("PointCloud saved to %s (%zu points)", ss.str().c_str(), pcl_cloud->size());

            save_pointcloud = false;

            // Reset for next scan
            clearVoxelGrid();
            ctn = 0;

            ROS_INFO("Ready for next scan. Waiting at home position...");
        }

        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
