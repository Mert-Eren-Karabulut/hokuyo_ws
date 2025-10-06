#include "segmentation.h"

#include <chrono>
#include <thread>

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segmentation_node");
    // Get the parameters from the launch file
    int numEstLPR, min_points_required_for_plane, minClusterSize;
    double seedsThreshold, distanceThreshold,
        plane_distance_threshold, vertical_threshold;
    // Segmentation
    double ellipsoidThreshold1, ellipsoidThreshold2, ellipsoidThreshold3,
        colorThreshold1, colorThreshold2, panEpsilon, tiltEpsilon, distEpsilon;
    std::string camera_point_topic, base_frame, output_namespace;

    ros::NodeHandle nh("~");
    nh.param("camera_point_topic", camera_point_topic, std::string("/zed/depth/points"));
    nh.param("base_frame", base_frame, std::string("odom"));
    nh.param("output_namespace", output_namespace, std::string("segmentation"));

    nh.param("numEstLPR", numEstLPR, 1000);
    nh.param("seedsThreshold", seedsThreshold, 0.01);
    nh.param("distanceThreshold", distanceThreshold, 0.01);
    nh.param("plane_distance_threshold", plane_distance_threshold, 0.01);
    nh.param("min_points_required_for_plane", min_points_required_for_plane, 1000);
    nh.param("vertical_threshold", vertical_threshold, 0.95);
    nh.param("ellipsoidThreshold1", ellipsoidThreshold1, 1.0);
    nh.param("ellipsoidThreshold2", ellipsoidThreshold2, 1.2);
    nh.param("ellipsoidThreshold3", ellipsoidThreshold3, 1.5);
    nh.param("colorThreshold1", colorThreshold1, 0.1);
    nh.param("colorThreshold2", colorThreshold2, 0.5);
    nh.param("panEpsilon", panEpsilon, 5.0);
    nh.param("tiltEpsilon", tiltEpsilon, 5.0);
    nh.param("distEpsilon", distEpsilon, 0.1);
    nh.param("minClusterSize", minClusterSize, 100);

    SegmentationParams params{
        camera_point_topic,
        base_frame,
        output_namespace,
        numEstLPR,
        seedsThreshold,
        distanceThreshold,
        plane_distance_threshold,
        min_points_required_for_plane,
        vertical_threshold,
        ellipsoidThreshold1,
        ellipsoidThreshold2,
        ellipsoidThreshold3,
        colorThreshold1,
        colorThreshold2,
        panEpsilon,
        tiltEpsilon,
        distEpsilon,
        minClusterSize};

    Segmentation segmentation(params);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Set the desired rate (e.g., 10 Hz)
    ros::Rate rate(10); // 10 Hz

    while (ros::ok())
    {
        // Your node's processing logic here
        segmentation.removeGround();
        segmentation.removeBackground();
        segmentation.segment();

        // Sleep to maintain the desired rate
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
