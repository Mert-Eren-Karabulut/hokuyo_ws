/*
 * Author       : Batuhan Vatan
 * Date         : 26 Aug '23
 * Description  : Simple APF (w/o obstacles) to track a person with a pan-tilt unit.
 */

#include "head_apf/head_apf.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "head_apf_node");

    float loopRate{};
    std::string target_topic_name;

    ros::NodeHandle nh("~");
    // get params from server
    nh.getParam("loop_rate", loopRate);
    nh.getParam("target_topic_name", target_topic_name);

    HeadAPF simpleTrack(loopRate, target_topic_name);

    while (ros::ok())
    {
        simpleTrack.apfStep();
        simpleTrack.printError();
        ros::spinOnce();
    }

    return 0;
}