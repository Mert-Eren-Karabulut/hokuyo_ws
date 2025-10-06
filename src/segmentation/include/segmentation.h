#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "colorhelper.h"

struct SegmentationParams
{
    std::string inputTopic = "/zed/depth/points";
    std::string baseFrame = "odom";
    std::string outputNamespace = "segmentation";
    int numEstLPR = 1000;
    double seedsThreshold = 0.01;
    double distanceThreshold = 0.01;
    double plane_distance_threshold = 0.01;
    int min_points_required_for_plane = 1000;
    double vertical_threshold = 0.95;
    double ellipsoidThreshold1 = 1.0;
    double ellipsoidThreshold2 = 1.2;
    double ellipsoidThreshold3 = 1.5;
    double colorThreshold1 = 0.1;
    double colorThreshold2 = 0.5;
    double panEpsilon = 5.0;
    double tiltEpsilon = 5.0;
    double distEpsilon = 0.1;
    int minClusterSize = 100;
};

class Segmentation
{
public:
    Segmentation(const SegmentationParams &params = {});

    void removeGround();

    void removeBackground();

    void segment();

private:
    SegmentationParams params_;

    // ROS
    ros::NodeHandle nh_;
    // Subscribers
    ros::Subscriber depthSub_;
    // Publishers
    ros::Publisher foregroundPub_;
    ros::Publisher obstaclesPub_;
    ros::Publisher coloredCloudPub_;

    // Point clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbdCloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obstacles_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground_;
    pcl::PointCloud<pcl::PointXYZRGB> sphereCloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud_;
    // Segmentation variables
    std::vector<bool> visited_;
    std::vector<int> clusters_;
    std::vector<HSV> colors_;
    std::vector<std::vector<int>> indexVector_;

    // Parameters
    // Ground elimination
    int numEstLPR_ = 1000;
    double seedsThreshold_ = 0.01;    // meters
    double distanceThreshold_ = 0.01; // meters
    // Background elimination
    double plane_distance_threshold_ = 0.01;
    int min_points_required_for_plane_ = 1000;
    double vertical_threshold_ = 0.95;
    // Segmentation
    double ellipsoidThreshold1_ = 1.0; // meters
    double ellipsoidThreshold2_ = 1.2; // meters
    double ellipsoidThreshold3_ = 1.5; // meters
    double colorThreshold1_ = 0.1;
    double colorThreshold2_ = 0.5;
    double panEpsilon_ = 5.0;  // degrees
    double tiltEpsilon_ = 5.0; // degrees
    double distEpsilon_ = 0.1; // meters
    int minClusterSize_ = 100;

    int panLimit_;
    int tiltLimit_;

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void toSphere();

    void checkNeighborhood(int index, int coreIndex, std::vector<int> &resultNeighbors);

    std::vector<int> regionQuery(int iterator, bool core);
};