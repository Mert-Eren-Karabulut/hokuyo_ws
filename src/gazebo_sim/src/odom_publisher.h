/*
Package name: minik_gazebo
File name: odom_publisher.h
Date created: 28.06.2020
Date last modified: 24.08.2020

Takes robot positions and headings with respect to world frame,
converts to them with respect to initial robot frames
 */

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>
#include <cfloat>
#define PI 3.14159265359
#define LOOP_RATE 10
#define N 5

using namespace std;

class OdomPublisher{
  public:

    OdomPublisher(int r);
    ~OdomPublisher();
    void work();



  private:


    ros::NodeHandle poseOdomSub;
    ros::NodeHandle posPub;
    ros::NodeHandle poseSub;

    //subscribers for posit,ons with respect to world frame for each robot

    void gazeboOdomCallback(const nav_msgs::Odometry::ConstPtr&);


    vector<double> calculatePos();

    int robot ;

    int n = N;
    double initPos[3];
    double pos[3];
    int flagN;
    int flag = 0;






};
