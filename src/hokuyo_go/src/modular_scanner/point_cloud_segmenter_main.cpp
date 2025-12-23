#include <ros/ros.h>
#include "hokuyo_go/modular_scanner/point_cloud_segmenter.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_segmenter");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO("Starting Point Cloud Segmenter Node...");
    
    hokuyo_go::PointCloudSegmenter segmenter(nh, pnh);
    
    ros::spin();
    
    return 0;
}
