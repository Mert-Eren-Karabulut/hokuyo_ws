#ifndef HOKUYO_GO_LASER_SCANNER_NODE_H
#define HOKUYO_GO_LASER_SCANNER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <chrono>

#include "hokuyo_go/modular_scanner/voxel_grid.h"
#include "hokuyo_go/modular_scanner/parametric_scanner.h"

namespace hokuyo_go
{

constexpr double PI = 3.14159265359;

class LaserScannerNode
{
public:
    LaserScannerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~LaserScannerNode();

    void run();

private:
    // ROS node handles
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    // Subscribers
    ros::Subscriber joint_sub_;
    ros::Subscriber laser_sub_;
    ros::Subscriber focus_point_sub_;

    // Publishers
    ros::Publisher joint_cmd_pub_;
    ros::Publisher pcl_pub_;
    ros::Publisher marker_pub_;

    // TF2 transform handling
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Voxel grid
    std::unique_ptr<VoxelGrid> voxel_grid_;
    float global_voxel_size_;

    // Parametric scanner
    std::unique_ptr<ParametricScanner> scanner_;

    // Current joint positions from Arduino
    double tiltangle_;
    double panangle_;
    bool joint_states_received_;

    //Current pattern limits
    double current_tilt_limit_max_;
    double current_pan_limit_max_;
    double current_tilt_limit_min_;
    double current_pan_limit_min_;

    // Angular velocity tracking
    double last_tiltangle_for_velocity_;
    double last_panangle_for_velocity_;
    std::chrono::high_resolution_clock::time_point last_velocity_time_;
    double path_speed_sum_;
    int path_speed_sample_count_;

    // Laser scan data
    std::vector<float> laser_scan_;
    sensor_msgs::LaserScan last_laser_;

    // Pointcloud variables
    sensor_msgs::PointCloud2 cloud_out_;
    int scan_count_;

    // Scanning control
    bool scanning_active_;
    bool save_pointcloud_;
    double scan_duration_;
    std::chrono::high_resolution_clock::time_point scan_start_time_;

    // Physical joint limits
    static constexpr double PAN_MAX = 150.0 * PI / 180.0;
    static constexpr double PAN_MIN = -150.0 * PI / 180.0;
    static constexpr double TILT_MAX = 45.0 * PI / 180.0;
    static constexpr double TILT_MIN = -60.0 * PI / 180.0;

    // Callbacks
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void focusPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg);

    // Helper functions
    void processLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg);
    void publishPointCloud();
    void publishJointCommand(double tilt_target, double pan_target);
    void savePointCloud();
    void placeDebugPoint(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b);
    bool checkPatternLimits(double phi_offset, double theta_offset, double delta_1, double delta_2);
    float calculateLocalizedSpeed();
};

} // namespace hokuyo_go

#endif // HOKUYO_GO_LASER_SCANNER_NODE_H
