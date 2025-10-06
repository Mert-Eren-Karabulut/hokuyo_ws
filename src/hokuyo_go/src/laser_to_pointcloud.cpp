#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/JointState.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <unordered_map>
#include <cmath>

using namespace std;

// Voxel structure for adaptive voxelization with arithmetic averaging
struct VoxelPoint {
    float x, y, z;          // Current averaged position
    float distance;         // Distance to closest point (for confidence)
    int count;              // Number of points accumulated
    
    VoxelPoint() : x(0), y(0), z(0), distance(0), count(0) {}
    VoxelPoint(float x_, float y_, float z_, float dist) 
        : x(x_), y(y_), z(z_), distance(dist), count(1) {}
    
    // Update position using simple arithmetic averaging
    void updatePosition(float new_x, float new_y, float new_z, float new_distance) {
        // Simple arithmetic average update (unity weight for all points)
        float new_count = count + 1;
        x = (x * count + new_x) / new_count;
        y = (y * count + new_y) / new_count;
        z = (z * count + new_z) / new_count;
        
        // Update count
        count++;
        
        // Update distance if new point is closer (for confidence tracking)
        if (new_distance < distance) {
            distance = new_distance;
        }
    }
};

// Voxel key for hash map
struct VoxelKey {
    int x, y, z;
    
    VoxelKey(int x_, int y_, int z_) : x(x_), y(y_), z(z_) {}
    
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

// Hash function for VoxelKey
struct VoxelKeyHash {
    size_t operator()(const VoxelKey& key) const {
        return hash<int>()(key.x) ^ (hash<int>()(key.y) << 1) ^ (hash<int>()(key.z) << 2);
    }
};

// Adaptive voxelization parameters from the paper
struct VoxelSizeRange {
    float min_distance;  // mm
    float max_distance;  // mm
    float voxel_size;    // mm
};

// Voxel size ranges based on Table 1 from the paper
const vector<VoxelSizeRange> VOXEL_RANGES = {
    {0.0f,    500.0f,  50.0f},   // 0-500mm: 50mm voxels
    {500.0f,  1000.0f, 100.0f},  // 500-1000mm: 100mm voxels  
    {1000.0f, 2000.0f, 200.0f},  // 1000-2000mm: 200mm voxels
    {2000.0f, 4000.0f, 400.0f}   // 2000-4000mm: 400mm voxels
};

// Voxel grid for adaptive point storage
unordered_map<VoxelKey, VoxelPoint, VoxelKeyHash> voxel_grid;

// Laser scan data
std::vector<float> laser_scan;
double laser_increment = 0.0;

// Joint state data
double tiltangle = 0.0;
double panangle = 0.0;
double lastangle = 0.0;
double lastpan = 0.0;

// TF2 transform handling
std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;

// Robot geometry constants - now only used for basic offset calculations
// With TF transforms, most of these become obsolete
const float start_angle = -2.094f;  // Laser start angle

// Processing variables
int scanSize = 0;
int ctn = 0;

// declaring the lasercan and pointcloud objects
sensor_msgs::LaserScan last_laser;
sensor_msgs::PointCloud2 cloud_out;

// Adaptive voxelization functions
float getVoxelSize(float distance_mm) {
    for (const auto& range : VOXEL_RANGES) {
        if (distance_mm >= range.min_distance && distance_mm < range.max_distance) {
            return range.voxel_size;
        }
    }
    // Return largest voxel size for distances beyond 4000mm
    return VOXEL_RANGES.back().voxel_size;
}

VoxelKey getVoxelKey(float x, float y, float z, float voxel_size) {
    // Convert world coordinates to voxel grid coordinates
    int vx = static_cast<int>(floor(x / voxel_size));
    int vy = static_cast<int>(floor(y / voxel_size));
    int vz = static_cast<int>(floor(z / voxel_size));
    return VoxelKey(vx, vy, vz);
}

void insertPointIntoVoxelGrid(float x, float y, float z, float distance) {
    // Convert distance from meters to millimeters
    float distance_mm = distance * 1000.0f;
    
    // Get appropriate voxel size for this distance
    float voxel_size = getVoxelSize(distance_mm);
    
    // Convert voxel size from mm to meters for coordinate calculations
    float voxel_size_m = voxel_size / 1000.0f;
    
    // Get voxel key
    VoxelKey key = getVoxelKey(x, y, z, voxel_size_m);
    
    // Check if voxel already exists
    auto it = voxel_grid.find(key);
    if (it != voxel_grid.end()) {
        // Voxel exists - update position using weighted averaging
        it->second.updatePosition(x, y, z, distance);
    } else {
        // New voxel - insert point
        voxel_grid[key] = VoxelPoint(x, y, z, distance);
    }
}

vector<VoxelPoint> getPointsFromVoxelGrid() {
    vector<VoxelPoint> points;
    points.reserve(voxel_grid.size());
    
    for (const auto& pair : voxel_grid) {
        points.push_back(pair.second);
    }
    
    return points;
}

void clearVoxelGrid() {
    voxel_grid.clear();
}

// Simple laser data callback - stores raw laser data for processing
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_increment = msg->angle_increment;
    laser_scan = msg->ranges;
    last_laser = *msg;
}

// Joint states callback with TF-based coordinate transformation
void encoCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    tiltangle = msg->position[1];
    panangle = msg->position[0];

    // Process points when there's significant movement
    if (fabs(tiltangle - lastangle) >= (M_PI)/180 || fabs(panangle - lastpan) >= (M_PI)/180)
    {
        if (!laser_scan.empty() && tf_buffer) {
            scanSize = static_cast<int>(laser_scan.size());
            
            for (int i = 0; i < scanSize; i++) {
                // Skip invalid measurements (beyond 4m range)
                if (laser_scan[i] <= 0.0 || laser_scan[i] > 4.0) continue;
                
                // Convert laser point to 2D coordinates in hokuyo_link frame
                float angle = laser_increment * i + start_angle;
                float x_laser = laser_scan[i] * cos(angle);
                float y_laser = laser_scan[i] * sin(angle);
                float z_laser = 0.0f;
                
                try {
                    // Create point in hokuyo_link frame
                    geometry_msgs::PointStamped point_hokuyo;
                    point_hokuyo.header.frame_id = "hokuyo_link";
                    point_hokuyo.header.stamp = last_laser.header.stamp;
                    point_hokuyo.point.x = x_laser;
                    point_hokuyo.point.y = y_laser;
                    point_hokuyo.point.z = z_laser;
                    
                    // Transform to base frame (taban)
                    geometry_msgs::PointStamped point_base;
                    tf_buffer->transform(point_hokuyo, point_base, "taban", ros::Duration(0.1));
                    
                    float x = point_base.point.x;
                    float y = point_base.point.y;
                    float z = point_base.point.z;
                    
                    // Calculate 3D distance for voxel size determination
                    float distance_3d = sqrt(x*x + y*y + z*z);
                    
                    // Insert point into adaptive voxel grid
                    insertPointIntoVoxelGrid(x, y, z, distance_3d);
                }
                catch (tf2::TransformException &ex) {
                    // Skip this point if transform fails
                    continue;
                }
            }
            
            ctn++;
            if (ctn % 50 == 0) {
                // Calculate average points per voxel for quality assessment
                float avg_points_per_voxel = 0.0f;
                if (!voxel_grid.empty()) {
                    float total_points = 0.0f;
                    for (const auto& pair : voxel_grid) {
                        total_points += pair.second.count;
                    }
                    avg_points_per_voxel = total_points / voxel_grid.size();
                }
                ROS_INFO("Processed %d scans, voxel grid: %lu voxels, avg %.1f points/voxel", 
                         ctn, voxel_grid.size(), avg_points_per_voxel);
            }
        }
        
        lastangle = tiltangle;
        lastpan = panangle;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_to_pointcloud");
    ros::NodeHandle n;

    // Initialize TF2 buffer and listener
    tf_buffer = std::make_unique<tf2_ros::Buffer>();
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

    // Parameters for voxelization control
    bool auto_clear_voxels = false;
    int max_voxel_count = 100000;  // Clear voxels when exceeding this count
    n.param("auto_clear_voxels", auto_clear_voxels, false);
    n.param("max_voxel_count", max_voxel_count, 100000);

    // subscribing topics
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, encoCallback);
    ros::Subscriber laserSub = n.subscribe("/hokuyo/scan", 1000, laserCallback); // hokuyo/scan

    // publishing resulting point cloud
    ros::Publisher pclPub = n.advertise<sensor_msgs::PointCloud2>("output", 10);

    ROS_INFO("Adaptive voxelization laser-to-pointcloud node started");
    ROS_INFO("Voxel size ranges:");
    for (const auto& range : VOXEL_RANGES) {
        ROS_INFO("  %.0f-%.0fmm: %.0fmm voxels", range.min_distance, range.max_distance, range.voxel_size);
    }
    ROS_INFO("Auto clear voxels: %s, Max voxel count: %d", 
             auto_clear_voxels ? "enabled" : "disabled", max_voxel_count);

    // publishing pointcloud
    ros::Rate loop(10);  // Reduced to 10Hz for better performance with voxelization
    while (ros::ok())
    {
        // Auto-clear voxels if enabled and limit exceeded
        if (auto_clear_voxels && voxel_grid.size() > static_cast<size_t>(max_voxel_count)) {
            ROS_INFO("Clearing voxel grid (size: %lu)", voxel_grid.size());
            clearVoxelGrid();
        }
        
        // Generate point cloud from voxel grid
        vector<VoxelPoint> voxel_points = getPointsFromVoxelGrid();
        
        if (!voxel_points.empty()) {
            // Setup point cloud message
            cloud_out.header.frame_id = "taban";
            cloud_out.header.stamp = last_laser.header.stamp;
            cloud_out.header.seq = cloud_out.header.seq + 1;
            
            // Configure point cloud structure
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
            for (size_t i = 0; i < voxel_points.size(); ++i) {
                float *pstep = (float *)&cloud_out.data[i * cloud_out.point_step];
                pstep[0] = voxel_points[i].x;
                pstep[1] = voxel_points[i].y;
                pstep[2] = voxel_points[i].z;
            }
        }

        pclPub.publish(cloud_out);
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
