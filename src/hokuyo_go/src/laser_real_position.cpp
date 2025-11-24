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
#include <limits>
#include <algorithm>

#define PI 3.14159265359

using namespace std;

// ========================================
// Voxel Structures (from laser_to_pointcloud.cpp)
// ========================================

/// Global Voxel Grid Parameters
#define MAX_SUBVOXEL_LEVEL 2      // Maximum subdivision level for multi-resolution voxels, size is (voxel_size / 2^max_level)
float SUBVOXEL_THRESHOLD = 0.50f; // Occupancy threshold to subdivide voxel further (configurable via ROS param)

// Forward declaration
struct SubVoxel;

// Voxel structure for global voxelization with arithmetic averaging
struct VoxelPoint
{
    float x, y, z;         // Current averaged position
    float sensor_distance; // Distance to sensor when point was acquired (for reference)
    int count;             // Number of points accumulated
    int subdivision_level; // Subdivision level (0 = parent, 1+ = subvoxel levels)

    VoxelPoint() : x(0), y(0), z(0), sensor_distance(0), count(0), subdivision_level(0) {}
    VoxelPoint(float x_, float y_, float z_, float dist, int level = 0)
        : x(x_), y(y_), z(z_), sensor_distance(dist), count(1), subdivision_level(level) {}

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

    // Get RGB color based on subdivision level
    void getColor(uint8_t &r, uint8_t &g, uint8_t &b) const
    {
        // Color scheme:
        // Level 0 (parent voxel, not subdivided): Blue
        // Level 1 (first subdivision): Green
        // Level 2 (second subdivision): Yellow
        // Level 3+ (max subdivision): Red

        switch (subdivision_level)
        {
        case 0:
            r = 50;
            g = 50;
            b = 255; // Blue
            break;
        case 1:
            r = 50;
            g = 255;
            b = 50; // Green
            break;
        case 2:
            r = 255;
            g = 255;
            b = 50; // Yellow
            break;
        case 3:
        default:
            r = 255;
            g = 50;
            b = 50; // Red
            break;
        }
    }
};

// SubVoxel structure for hierarchical subdivision
struct SubVoxel
{
    VoxelPoint point_data;          // Averaged point data for this subvoxel
    std::vector<SubVoxel> children; // Child subvoxels (8 children for octree)
    int current_level;              // Current subdivision level
    bool is_subdivided;             // Whether this subvoxel has been subdivided
    float min_x, min_y, min_z;      // Bounding box min
    float max_x, max_y, max_z;      // Bounding box max

    SubVoxel() : current_level(0), is_subdivided(false),
                 min_x(0), min_y(0), min_z(0),
                 max_x(0), max_y(0), max_z(0) {}

    SubVoxel(float minx, float miny, float minz, float maxx, float maxy, float maxz, int level)
        : current_level(level), is_subdivided(false),
          min_x(minx), min_y(miny), min_z(minz),
          max_x(maxx), max_y(maxy), max_z(maxz) {}

    // Check if a point is within this subvoxel's bounds
    bool containsPoint(float x, float y, float z) const
    {
        return x >= min_x && x < max_x &&
               y >= min_y && y < max_y &&
               z >= min_z && z < max_z;
    }

    // Get the child index for a point (octree indexing: 0-7)
    int getChildIndex(float x, float y, float z) const
    {
        float mid_x = (min_x + max_x) / 2.0f;
        float mid_y = (min_y + max_y) / 2.0f;
        float mid_z = (min_z + max_z) / 2.0f;

        int idx = 0;
        if (x >= mid_x)
            idx |= 1;
        if (y >= mid_y)
            idx |= 2;
        if (z >= mid_z)
            idx |= 4;
        return idx;
    }

    // Create 8 children subvoxels
    void subdivide()
    {
        if (is_subdivided || current_level >= MAX_SUBVOXEL_LEVEL)
            return;

        float mid_x = (min_x + max_x) / 2.0f;
        float mid_y = (min_y + max_y) / 2.0f;
        float mid_z = (min_z + max_z) / 2.0f;

        children.resize(8);
        int next_level = current_level + 1;

        // Create 8 octants
        children[0] = SubVoxel(min_x, min_y, min_z, mid_x, mid_y, mid_z, next_level);
        children[1] = SubVoxel(mid_x, min_y, min_z, max_x, mid_y, mid_z, next_level);
        children[2] = SubVoxel(min_x, mid_y, min_z, mid_x, max_y, mid_z, next_level);
        children[3] = SubVoxel(mid_x, mid_y, min_z, max_x, max_y, mid_z, next_level);
        children[4] = SubVoxel(min_x, min_y, mid_z, mid_x, mid_y, max_z, next_level);
        children[5] = SubVoxel(mid_x, min_y, mid_z, max_x, mid_y, max_z, next_level);
        children[6] = SubVoxel(min_x, mid_y, mid_z, mid_x, max_y, max_z, next_level);
        children[7] = SubVoxel(mid_x, mid_y, mid_z, max_x, max_y, max_z, next_level);

        is_subdivided = true;
    }

    // Remove subdivision and consolidate points
    void unsubdivide()
    {
        if (!is_subdivided)
            return;

        // Merge all child points + existing parent points into consolidated average
        int total_count = 0;
        float total_x = 0, total_y = 0, total_z = 0;
        float min_distance = std::numeric_limits<float>::max();

        // Include existing parent point_data (points from before subdivision)
        if (point_data.count > 0)
        {
            total_x += point_data.x * point_data.count;
            total_y += point_data.y * point_data.count;
            total_z += point_data.z * point_data.count;
            total_count += point_data.count;
            min_distance = point_data.sensor_distance;
        }

        // Include all child points
        for (auto &child : children)
        {
            if (child.point_data.count > 0)
            {
                total_x += child.point_data.x * child.point_data.count;
                total_y += child.point_data.y * child.point_data.count;
                total_z += child.point_data.z * child.point_data.count;
                total_count += child.point_data.count;
                min_distance = std::min(min_distance, child.point_data.sensor_distance);
            }
        }

        if (total_count > 0)
        {
            point_data.x = total_x / total_count;
            point_data.y = total_y / total_count;
            point_data.z = total_z / total_count;
            point_data.count = total_count;
            point_data.sensor_distance = min_distance;
        }

        children.clear();
        is_subdivided = false;
    }
};

// Parent voxel structure containing subvoxels
struct ParentVoxel
{
    std::vector<SubVoxel> subvoxels; // 8 subvoxels
    VoxelPoint averaged_point;       // Fallback averaged point if not subdivided
    bool is_subdivided;              // Whether this voxel is using subvoxels
    float min_x, min_y, min_z;       // Voxel bounds
    float max_x, max_y, max_z;
    int total_points; // Total points in this parent voxel

    ParentVoxel() : is_subdivided(false),
                    min_x(0), min_y(0), min_z(0),
                    max_x(0), max_y(0), max_z(0),
                    total_points(0) {}

    ParentVoxel(float minx, float miny, float minz, float voxel_size)
        : is_subdivided(false),
          min_x(minx), min_y(miny), min_z(minz),
          max_x(minx + voxel_size), max_y(miny + voxel_size), max_z(minz + voxel_size),
          total_points(0)
    {
        // Initialize 8 first-level subvoxels
        subvoxels.resize(8);
        float mid_x = (min_x + max_x) / 2.0f;
        float mid_y = (min_y + max_y) / 2.0f;
        float mid_z = (min_z + max_z) / 2.0f;

        subvoxels[0] = SubVoxel(min_x, min_y, min_z, mid_x, mid_y, mid_z, 1);
        subvoxels[1] = SubVoxel(mid_x, min_y, min_z, max_x, mid_y, mid_z, 1);
        subvoxels[2] = SubVoxel(min_x, mid_y, min_z, mid_x, max_y, mid_z, 1);
        subvoxels[3] = SubVoxel(mid_x, mid_y, min_z, max_x, max_y, mid_z, 1);
        subvoxels[4] = SubVoxel(min_x, min_y, mid_z, mid_x, mid_y, max_z, 1);
        subvoxels[5] = SubVoxel(mid_x, min_y, mid_z, max_x, mid_y, max_z, 1);
        subvoxels[6] = SubVoxel(min_x, mid_y, mid_z, mid_x, max_y, max_z, 1);
        subvoxels[7] = SubVoxel(mid_x, mid_y, mid_z, max_x, max_y, max_z, 1);
    }

    // Insert point and manage subdivision
    void insertPoint(float x, float y, float z, float distance)
    {
        total_points++;

        // Always update averaged point as fallback
        averaged_point.updatePosition(x, y, z, distance);

        // Determine which first-level subvoxel this point belongs to
        int subvoxel_idx = getSubvoxelIndex(x, y, z);
        if (subvoxel_idx < 0 || subvoxel_idx >= 8)
            return;

        SubVoxel &subvox = subvoxels[subvoxel_idx];
        insertIntoSubvoxel(subvox, x, y, z, distance);

        // Only evaluate subdivision periodically (every 10 points) to avoid thrashing
        if (total_points % 10 == 0)
        {
            evaluateSubdivision();
        }
    }

    int getSubvoxelIndex(float x, float y, float z) const
    {
        float mid_x = (min_x + max_x) / 2.0f;
        float mid_y = (min_y + max_y) / 2.0f;
        float mid_z = (min_z + max_z) / 2.0f;

        int idx = 0;
        if (x >= mid_x)
            idx |= 1;
        if (y >= mid_y)
            idx |= 2;
        if (z >= mid_z)
            idx |= 4;
        return idx;
    }

    void insertIntoSubvoxel(SubVoxel &subvox, float x, float y, float z, float distance)
    {
        if (subvox.is_subdivided)
        {
            // Recursively insert into appropriate child
            int child_idx = subvox.getChildIndex(x, y, z);
            if (child_idx >= 0 && child_idx < 8)
            {
                insertIntoSubvoxel(subvox.children[child_idx], x, y, z, distance);
            }
        }
        else
        {
            // Add to this subvoxel's averaged point
            subvox.point_data.updatePosition(x, y, z, distance);
        }
    }

    void evaluateSubdivision()
    {
        // Determine if parent should be subdivided based on subvoxel distribution
        int subvoxels_above_threshold = 0;
        for (const auto &subvox : subvoxels)
        {
            float ratio = (total_points > 0) ? (float)subvox.point_data.count / (float)total_points : 0.0f;
            if (ratio >= SUBVOXEL_THRESHOLD)
            {
                subvoxels_above_threshold++;
            }
        }

        is_subdivided = (subvoxels_above_threshold > 0);

        // Now evaluate each first-level subvoxel for further subdivision
        for (auto &subvox : subvoxels)
        {
            evaluateSubvoxelSubdivision(subvox);
        }
    }

    void evaluateSubvoxelSubdivision(SubVoxel &subvox)
    {
        if (subvox.current_level >= MAX_SUBVOXEL_LEVEL)
            return;

        // A subvoxel should be subdivided if it has enough points
        // and those points are concentrated in one area (ratio >= threshold)
        if (subvox.point_data.count == 0)
            return;

        if (!subvox.is_subdivided)
        {
            // Check if this subvoxel has significant concentration relative to parent
            float ratio = (total_points > 0) ? (float)subvox.point_data.count / (float)total_points : 0.0f;

            // Require more points at deeper levels and enough total points
            int min_points_to_subdivide = 5 * (subvox.current_level + 1);

            if (ratio >= SUBVOXEL_THRESHOLD && subvox.point_data.count >= min_points_to_subdivide)
            {
                // Subdivide this subvoxel
                subvox.subdivide();
                // Note: Existing averaged point stays in point_data
                // New points will be distributed to children
            }
        }
        else
        {
            // Already subdivided, check if we should keep it subdivided
            // Count points in children
            int child_points = 0;
            int children_with_points = 0;
            int max_child_points = 0;

            for (const auto &child : subvox.children)
            {
                int cp = child.point_data.count;
                child_points += cp;
                if (cp > 0)
                    children_with_points++;
                if (cp > max_child_points)
                    max_child_points = cp;
            }

            // If we have child points, check concentration
            if (child_points > 0)
            {
                float max_child_ratio = (float)max_child_points / (float)child_points;

                // Keep subdivided if at least one child has significant concentration
                if (max_child_ratio >= SUBVOXEL_THRESHOLD)
                {
                    // Recursively evaluate children that have enough points
                    for (auto &child : subvox.children)
                    {
                        if (child.point_data.count >= 3) // minimum points for further subdivision
                        {
                            evaluateSubvoxelSubdivisionRecursive(child, child_points);
                        }
                    }
                }
                else
                {
                    // No concentration, unsubdivide
                    subvox.unsubdivide();
                }
            }
        }
    }

    // Recursive helper that uses the correct parent total for ratio calculation
    void evaluateSubvoxelSubdivisionRecursive(SubVoxel &subvox, int parent_total)
    {
        if (subvox.current_level >= MAX_SUBVOXEL_LEVEL)
            return;

        if (subvox.point_data.count == 0)
            return;

        if (!subvox.is_subdivided)
        {
            // Check if this subvoxel should be subdivided based on its share of parent's points
            float ratio = (parent_total > 0) ? (float)subvox.point_data.count / (float)parent_total : 0.0f;

            // Need fewer points at deeper levels since voxels are smaller
            int min_points = 3 * (subvox.current_level + 1);

            if (ratio >= SUBVOXEL_THRESHOLD && subvox.point_data.count >= min_points)
            {
                subvox.subdivide();
            }
        }
        else
        {
            // Check children concentration
            int child_points = 0;
            int max_child_points = 0;

            for (const auto &child : subvox.children)
            {
                int cp = child.point_data.count;
                child_points += cp;
                if (cp > max_child_points)
                    max_child_points = cp;
            }

            if (child_points > 0)
            {
                float max_child_ratio = (float)max_child_points / (float)child_points;

                if (max_child_ratio >= SUBVOXEL_THRESHOLD)
                {
                    // Keep subdivided and recurse
                    for (auto &child : subvox.children)
                    {
                        if (child.point_data.count >= 2)
                        {
                            evaluateSubvoxelSubdivisionRecursive(child, child_points);
                        }
                    }
                }
                else
                {
                    subvox.unsubdivide();
                }
            }
        }
    }

    // Collect all leaf points for visualization
    void collectPoints(std::vector<VoxelPoint> &points) const
    {
        if (!is_subdivided)
        {
            // Parent is not subdivided - use parent-level averaged point (level 0)
            // This means none of the subvoxels reached the threshold
            if (averaged_point.count > 0)
            {
                VoxelPoint pt = averaged_point;
                pt.subdivision_level = 0; // Parent level
                points.push_back(pt);
            }
        }
        else
        {
            // Parent is subdivided - collect from first-level subvoxels
            // At least one subvoxel exceeded the threshold
            for (const auto &subvox : subvoxels)
            {
                collectSubvoxelPoints(subvox, points);
            }
        }
    }

    void collectSubvoxelPoints(const SubVoxel &subvox, std::vector<VoxelPoint> &points) const
    {
        if (subvox.is_subdivided)
        {
            // This subvoxel is subdivided - recursively collect from children ONLY
            // The parent's point_data is kept for potential unsubdivide, but not rendered
            // to avoid double-counting (parent represents pre-subdivision data)
            for (const auto &child : subvox.children)
            {
                collectSubvoxelPoints(child, points);
            }
        }
        else
        {
            // This subvoxel is a leaf - add its averaged point with its level
            if (subvox.point_data.count > 0)
            {
                VoxelPoint pt = subvox.point_data;
                pt.subdivision_level = subvox.current_level;
                points.push_back(pt);
            }
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
unordered_map<VoxelKey, ParentVoxel, VoxelKeyHash> voxel_grid;

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
double delta_1 = 30.0 * PI / 180.0;        // Pan limit (60 degrees)
double delta_2 = 30.0 * PI / 180.0;        // Tilt limit (45 degrees)
double sqrt2_over_100 = sqrt(2.0) / 100.0; // Irrational frequency component
auto last_param_update_time = std::chrono::high_resolution_clock::now();
double phi_offset = 0.0;                    // Phase offset for scanning pattern
double theta_offset = 0.0;                  // Additional tilt offset
#define PAN_MAX 150.0 * PI / 180.0          // Physical pan limit
#define PAN_MIN -150.0 * PI / 180.0         //
#define TILT_MAX 45.0 * PI / 180.0          // Physical tilt limit
#define TILT_MIN -60.0 * PI / 180.0         //
double current_pattern_pan_max = delta_1;   // Current pan limit for pattern
double current_pattern_pan_min = -delta_1;  // Current pan limit for pattern
double current_pattern_tilt_max = delta_2;  // Current tilt limit for pattern
double current_pattern_tilt_min = -delta_2; // Current tilt limit for pattern

// Laser scan data
std::vector<float> laser_scan;
sensor_msgs::LaserScan last_laser;
double laser_increment = 0.0;

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
        // Insert into existing parent voxel
        it->second.insertPoint(x, y, z, distance);
    }
    else
    {
        // Create new parent voxel
        float voxel_min_x = key.x * GLOBAL_VOXEL_SIZE;
        float voxel_min_y = key.y * GLOBAL_VOXEL_SIZE;
        float voxel_min_z = key.z * GLOBAL_VOXEL_SIZE;

        ParentVoxel parent_voxel(voxel_min_x, voxel_min_y, voxel_min_z, GLOBAL_VOXEL_SIZE);
        parent_voxel.insertPoint(x, y, z, distance);
        voxel_grid[key] = parent_voxel;
    }
}

vector<VoxelPoint> getPointsFromVoxelGrid()
{
    vector<VoxelPoint> points;

    // Estimate capacity based on voxel grid size
    size_t estimated_size = voxel_grid.size() * 4; // rough estimate
    points.reserve(estimated_size);

    for (const auto &pair : voxel_grid)
    {
        pair.second.collectPoints(points);
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
    return phi_offset + delta_1 * sin(t);
}

double f2(double t)
{
    // Tilt function: non-repetitive scanning pattern
    double freq = 3.0 + sqrt2_over_100;
    return theta_offset + (delta_2 * 2) * (cos(freq * t) + 1.0) / 2.0 - delta_2;
    return 0.0;
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
    laser_increment = 0.36;
    // from ranges[44] inclusive to ranges[724] inclusive, total 681 points
    laser_scan = vector<float>(msg->ranges.begin() + 44, msg->ranges.begin() + 725);
    last_laser = *msg;

    // Process laser scan into voxel grid immediately if scanning is active
    if (scanning_active && !laser_scan.empty() && tf_buffer && joint_states_received)
    {
        scanSize = static_cast<int>(laser_scan.size());

        // Use message-provided angles/timings
        const float angle_min = -2.094395102;
        const float angle_inc = 0.0061359232;
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

            // Point in laser frame
            geometry_msgs::PointStamped pt_laser;
            pt_laser.header.frame_id = "laser";
            pt_laser.header.stamp = msg->header.stamp;
            pt_laser.point.x = r * std::cos(angle);
            pt_laser.point.y = r * std::sin(angle);
            pt_laser.point.z = 0.0;

            // Quick transform guard to avoid exceptions
            if (!tf_buffer->canTransform("map", "laser", pt_laser.header.stamp, ros::Duration(0.02)))
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
            int total_leaf_voxels = 0;
            int subdivided_voxels = 0;
            int total_parent_voxels = voxel_grid.size();

            vector<VoxelPoint> leaf_points; // declared here so debug reporting below can access it
            if (!voxel_grid.empty())
            {
                float total_points = 0.0f;
                for (const auto &pair : voxel_grid)
                {
                    total_points += pair.second.total_points;
                    if (pair.second.is_subdivided)
                    {
                        subdivided_voxels++;
                    }
                }

                // Count actual leaf voxels (for visualization)
                leaf_points = getPointsFromVoxelGrid();
                total_leaf_voxels = leaf_points.size();

                if (total_leaf_voxels > 0)
                {
                    avg_points_per_voxel = total_points / total_leaf_voxels;
                }
            }

            // Calculate average path-following speed for this interval
            double avg_path_speed = 0.0;
            if (path_speed_sample_count > 0)
            {
                avg_path_speed = path_speed_sum / path_speed_sample_count;
            }

            ROS_INFO("Scans: %d | Parent voxels: %d | Subdivided: %d | Leaf voxels: %d (avg %.1f pts/leaf) | Path: %.3f rad/s",
                     ctn, total_parent_voxels, subdivided_voxels, total_leaf_voxels,
                     avg_points_per_voxel, avg_path_speed);

            // Debug: Count points by subdivision level
            int level_counts[4] = {0, 0, 0, 0};
            for (const auto &pt : leaf_points)
            {
                int level = std::min(pt.subdivision_level, 3);
                level_counts[level]++;
            }
            ROS_INFO("  Level distribution: L0(blue)=%d, L1(green)=%d, L2(yellow)=%d, L3+(red)=%d",
                     level_counts[0], level_counts[1], level_counts[2], level_counts[3]);

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

void focusPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    ROS_INFO("Focus point received at (%.2f, %.2f, %.2f) in frame %s",
             msg->point.x, msg->point.y, msg->point.z, msg->header.frame_id.c_str());

    // IK Solution Strategy:
    // 1. Transform target point to current laser frame
    // 2. In laser frame, target direction tells us how to adjust pan/tilt
    // 3. Laser X+ should point at target, so we need pan/tilt offsets based on Y and Z components

    try
    {
        // Transform target to current laser frame
        geometry_msgs::PointStamped pt_laser;
        if (!tf_buffer->canTransform("laser", msg->header.frame_id, msg->header.stamp, ros::Duration(0.1)))
        {
            ROS_WARN("Cannot transform focus point to laser frame");
            return;
        }

        tf_buffer->transform(*msg, pt_laser, "laser", ros::Duration(0.1));

        double lx = pt_laser.point.x; // Along laser's optical axis (desired direction)
        double ly = pt_laser.point.y; // Perpendicular to optical axis (pan adjustment needed)
        double lz = pt_laser.point.z; // Perpendicular to optical axis (tilt adjustment needed)

        ROS_INFO("Target in current laser frame: (%.3f, %.3f, %.3f)", lx, ly, lz);

        // Calculate the direction from laser origin to target in laser frame
        double range = sqrt(lx * lx + ly * ly + lz * lz);

        if (range < 0.01)
        {
            ROS_WARN("Target too close to laser origin, ignoring");
            return;
        }

        // To make the laser's X+ axis point at the target, we need to adjust pan/tilt
        // such that the target moves to lie along the X+ axis (ly=0, lz=0)

        // Current angles
        double current_pan = panangle;
        double current_tilt = tiltangle;

        ROS_INFO("Current joint angles: Pan = %.2f deg, Tilt = %.2f deg",
                 current_pan * 180.0 / PI, current_tilt * 180.0 / PI);

        // Required adjustments in laser frame coordinates:
        // Pan adjustment: rotate to eliminate Y component
        // Tilt adjustment: rotate to eliminate Z component

        // Pan adjustment needed (rotation around vertical axis in world)
        // In laser frame, Y component indicates how much pan adjustment is needed
        double delta_pan = atan2(ly, lx);

        // Tilt adjustment needed (rotation around horizontal axis)
        // Z component indicates how much tilt adjustment is needed
        double delta_tilt = atan2(lz, lx);

        // Note: The sign and exact relationship depend on the kinematic chain
        // Pan joint has inverted axis (0 0 -1), so we may need to negate
        // Let's compute the target angles:

        double phi_solved = current_pan - delta_pan;     // Subtract because pan axis is inverted
        double theta_solved = current_tilt + delta_tilt; // Add for tilt

        // Check if target is behind the laser (negative X)
        // If behind, we'll add 180° to pan later to turn around
        bool target_behind = (lx < 0.0);

        // If target was behind, add 180° to pan to actually turn around
        if (target_behind)
        {

            phi_solved += PI; // Add 180 degrees to turn around
            ROS_INFO("Added 180° to pan for rear-facing target");
        }

        // Normalize angles to [-PI, PI] range
        // This handles cases like -288° -> +72°
        while (phi_solved > PI)
            phi_solved -= 2.0 * PI;
        while (phi_solved < -PI)
            phi_solved += 2.0 * PI;
        while (theta_solved > PI)
            theta_solved -= 2.0 * PI;
        while (theta_solved < -PI)
            theta_solved += 2.0 * PI;

        ROS_INFO("IK Solution (normalized): Pan = %.2f deg, Tilt = %.2f deg",
                 phi_solved * 180.0 / PI, theta_solved * 180.0 / PI);
        ROS_INFO("  Adjustments from current: ΔPan = %.2f deg, ΔTilt = %.2f deg",
                 (phi_solved - current_pan) * 180.0 / PI, (theta_solved - current_tilt) * 180.0 / PI);

        // Calculate pattern limits if we use this IK solution as the center
        double pan_limit_max = phi_solved + delta_1;
        double tilt_limit_max = theta_solved + delta_2;
        double pan_limit_min = phi_solved - delta_1;
        double tilt_limit_min = theta_solved - delta_2;

        ROS_INFO("Proposed pattern limits: Pan [%.2f, %.2f] deg, Tilt [%.2f, %.2f] deg",
                 pan_limit_min * 180.0 / PI, pan_limit_max * 180.0 / PI,
                 tilt_limit_min * 180.0 / PI, tilt_limit_max * 180.0 / PI);

        // Check if pattern limits exceed physical joint limits
        bool limits_exceeded = false;

        if (pan_limit_max > PAN_MAX)
        {
            ROS_WARN("Pattern exceeds pan max limit: %.2f > %.2f deg",
                     pan_limit_max * 180.0 / PI, PAN_MAX * 180.0 / PI);
            limits_exceeded = true;
        }
        if (pan_limit_min < PAN_MIN)
        {
            ROS_WARN("Pattern exceeds pan min limit: %.2f < %.2f deg",
                     pan_limit_min * 180.0 / PI, PAN_MIN * 180.0 / PI);
            limits_exceeded = true;
        }
        if (tilt_limit_max > TILT_MAX)
        {
            ROS_WARN("Pattern exceeds tilt max limit: %.2f > %.2f deg",
                     tilt_limit_max * 180.0 / PI, TILT_MAX * 180.0 / PI);
            limits_exceeded = true;
        }
        if (tilt_limit_min < TILT_MIN)
        {
            ROS_WARN("Pattern exceeds tilt min limit: %.2f < %.2f deg",
                     tilt_limit_min * 180.0 / PI, TILT_MIN * 180.0 / PI);
            limits_exceeded = true;
        }

        // Apply the IK solution as offsets if within limits
        if (limits_exceeded)
        {
            ROS_WARN("Cannot center pattern on focus point - would exceed physical limits");
            ROS_WARN("Consider clicking a point closer to the current scan center");
        }
        else
        {
            ROS_INFO("=== UPDATING SCAN CENTER ===");
            ROS_INFO("Old offsets: Pan = %.2f deg, Tilt = %.2f deg",
                     phi_offset * 180.0 / PI, theta_offset * 180.0 / PI);

            // Set the IK solution as the new pattern center offsets
            phi_offset = phi_solved;
            theta_offset = theta_solved;

            // Update pattern limits
            current_pattern_pan_max = pan_limit_max;
            current_pattern_pan_min = pan_limit_min;
            current_pattern_tilt_max = tilt_limit_max;
            current_pattern_tilt_min = tilt_limit_min;

            ROS_INFO("New offsets: Pan = %.2f deg, Tilt = %.2f deg",
                     phi_offset * 180.0 / PI, theta_offset * 180.0 / PI);
            ROS_INFO("Scan pattern will now be centered on the selected point");
        }
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Transform exception: %s", ex.what());
        return;
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
    nh.param("scan_duration", scan_duration, 0.0);

    // Voxelization parameters
    double subvoxel_threshold_param = 0.40; // default 50%
    nh.param("subvoxel_threshold", subvoxel_threshold_param, 0.40);
    SUBVOXEL_THRESHOLD = static_cast<float>(subvoxel_threshold_param);

    // Convert degrees to radians
    // delta_1 = delta_1 * PI / 180.0;
    // delta_2 = delta_2 * PI / 180.0;

    ROS_INFO("=== Parametric Position-Based Laser Scanner with TF Voxelization ===");
    ROS_INFO("Target velocity: %.2f rad/s", v_target);
    ROS_INFO("Pan limit: ±%.1f degrees", delta_1 * 180.0 / PI);
    ROS_INFO("Tilt limit: ±%.1f degrees", delta_2 * 180.0 / PI);
    ROS_INFO("Scan duration: %.1f seconds", scan_duration);
    ROS_INFO("Using TF transforms instead of hardcoded kinematics");
    ROS_INFO("Voxel size: %.0fmm | Subvoxel threshold: %.0f%% | Max levels: %d",
             GLOBAL_VOXEL_SIZE * 1000.0f, SUBVOXEL_THRESHOLD * 100.0f, MAX_SUBVOXEL_LEVEL);

    // Subscribers
    ros::Subscriber joint_sub = nh.subscribe("/joint_states", 100, jointStateCallback);
    ros::Subscriber laser_sub = nh.subscribe("/scan", 100, laserCallback);
    ros::Subscriber focus_point_sub = nh.subscribe("/clicked_point", 10, focusPointCallback);

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
            // Setup point cloud message with RGB color
            cloud_out.header.frame_id = "map";
            cloud_out.header.stamp = ros::Time::now();
            cloud_out.height = 1;
            cloud_out.width = voxel_points.size();
            cloud_out.fields.resize(4);

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

            cloud_out.fields[3].name = "rgb";
            cloud_out.fields[3].offset = 12;
            cloud_out.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
            cloud_out.fields[3].count = 1;

            cloud_out.point_step = 16;
            cloud_out.row_step = cloud_out.point_step * cloud_out.width;
            cloud_out.data.resize(cloud_out.row_step * cloud_out.height);
            cloud_out.is_dense = false;

            // Fill point cloud data from voxel grid with colors
            for (size_t i = 0; i < voxel_points.size(); ++i)
            {
                float *pstep = (float *)&cloud_out.data[i * cloud_out.point_step];
                pstep[0] = voxel_points[i].x;
                pstep[1] = voxel_points[i].y;
                pstep[2] = voxel_points[i].z;

                // Get color based on subdivision level
                uint8_t r, g, b;
                voxel_points[i].getColor(r, g, b);

                // Pack RGB into a single float (as used by PCL)
                uint32_t rgb_packed = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
                pstep[3] = *reinterpret_cast<float *>(&rgb_packed);
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
