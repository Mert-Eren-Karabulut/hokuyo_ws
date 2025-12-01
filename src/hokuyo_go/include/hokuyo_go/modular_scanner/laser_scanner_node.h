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
#include <vector>
#include <map>

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

    // Observation phase settings
    static constexpr double OBSERVATION_DURATION = 10.0;  // seconds
    static constexpr double HISTOGRAM_MIN_DISTANCE = 2.0; // meters, only consider points beyond this
    static constexpr int NUM_HISTOGRAM_BINS = 50;         // number of bins for histogram
    
    // Observation phase state
    bool observation_phase_active_;
    double delta_1_observe_;  // observing pan limit (radians)
    double delta_2_observe_;  // observing tilt limit (radians)
    
    // Spherical point cloud storage (r, pan, tilt)
    struct SphericalPoint {
        float r;      // distance
        float pan;    // azimuth angle
        float tilt;   // elevation angle
    };
    std::vector<SphericalPoint> spherical_points_;
    
    // Histogram data
    struct HistogramBin {
        float r_min;
        float r_max;
        int count;
    };
    std::vector<HistogramBin> distance_histogram_;
    
    // Distance intervals (dense zones)
    struct DistanceInterval {
        float r_min;
        float r_max;
        int point_count;
    };
    std::vector<DistanceInterval> dense_intervals_;

    // Scan zone structure for focused scanning
    struct ScanZone {
        float center_x, center_y, center_z;  // Center point in map frame
        float pan_center, tilt_center;        // Center in spherical coords
        float delta_pan, delta_tilt;          // Calculated scan deltas
        float r_avg;                          // Average distance
        int point_count;                      // Number of points in zone
        int interval_idx;                     // Which interval this zone belongs to
    };
    std::vector<ScanZone> scan_zones_;
    
    // Focused scanning state
    static constexpr double ZONE_SCAN_DURATION = 5.0;  // seconds per zone
    bool focused_scanning_active_;
    int current_interval_idx_;
    int current_zone_idx_;
    std::chrono::high_resolution_clock::time_point zone_scan_start_time_;

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
    
    // Observation phase and histogram functions
    void storeSphericalPoint(float x, float y, float z);
    void convertVoxelsToSpherical();
    void computeDistanceHistogram();
    void findDenseIntervals(int num_intervals);
    void reportHistogram();
    void transitionToNormalMode();
    
    // Focused scanning functions
    std::vector<SphericalPoint> getPointsInInterval(float r_min, float r_max);
    void clusterPointsIntoZones(const std::vector<SphericalPoint>& points, int interval_idx);
    void calculateZoneScanParameters(ScanZone& zone);
    void startFocusedScanning();
    void processZoneScanning();
    bool startNextZoneScan();
    void transitionToManualMode();
    
    // IK helper - computes joint angles to point at a target in map frame
    bool computeJointAnglesForTarget(double x, double y, double z, double& pan_out, double& tilt_out);
    
    // Visualization
    void publishZoneVisualization(const ScanZone& zone, float r_min, float r_max);
    void clearZoneVisualization();
};

} // namespace hokuyo_go

#endif // HOKUYO_GO_LASER_SCANNER_NODE_H
