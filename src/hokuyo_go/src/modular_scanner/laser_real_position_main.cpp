// Main entry point for the laser scanner node
#include <ros/ros.h>
#include "hokuyo_go/modular_scanner/laser_scanner_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_real_position");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    hokuyo_go::LaserScannerNode node(nh, pnh);
    node.run();

    return 0;
}
