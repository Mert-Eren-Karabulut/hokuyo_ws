/*
Package name: minik_gazebo
File name: odom_publisher.cpp
Author: Berkay Gumus
E-mail: berkay.gumus@boun.edu.tr
Date created: 28.06.2020
Date last modified: 24.08.2020

Takes robot positions and headings with respect to world frame,
converts to them with respect to initial robot frames
*/

#include "odom_publisher.h"
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>


using namespace std;



OdomPublisher::OdomPublisher(int r){
    this->robot = r;
    cout << "Robot odometry " << r  <<  endl;

}

OdomPublisher::~OdomPublisher(){
}


//subscriber for robot 1 position

void OdomPublisher::gazeboOdomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  //position with respect to world frame: x,y
  double temp_x = msg->pose.pose.position.x; double temp_y = msg->pose.pose.position.y;

  //quaternion
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  //rotation angles
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //related angle is yaw >> heading with respect to world frame
  double temp_theta = yaw;

  //to save initial position and heading
  if(flag==0){
    initPos[0] = temp_x;
    initPos[1] = temp_y;
    initPos[2] = temp_theta;//orientation2theta(0, 0, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  }
  //current position and heading
  pos[0] = temp_x;
  pos[1] = temp_y;
  pos[2] = temp_theta;//orientation2theta(0, 0, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  flagN=1; //check whether initial position and heading are saved for robot 1

}


vector<double> OdomPublisher::calculatePos(){
  //Takes robot id, converts its pos wrt world frame to wrt initial frame

  //initial robot frame
  double initX = initPos[0];
  double initY = initPos[1];
  double initTheta = initPos[2];

  // position wrt world frame
  double posX0 = pos[0];
  double posY0 = pos[1];
  double posTheta0 = pos[2];

  //initialization of position wrt frame robot r
  vector<double> posR;
  posR.resize(3);


  //rotation matrix
  double rot[3][3] = {{cos(initTheta),sin(initTheta),0},{-sin(initTheta),cos(initTheta),0},{0,0,1}};
  //translation vector
  double trans[3] = {posX0 - initX, posY0-initY,1};

  //position and heading wrt initial robot frame
  posR[0] = rot[0][0] * trans[0] + rot[0][1] * trans[1];
  posR[1] = rot[1][0] * trans[0] + rot[1][1] * trans[1];
  posR[2] = posTheta0 - initTheta;

  //cout << i <<"-->" << j << " pos: " << posR[0] << " " << posR[1] << endl;

  //return position and heading wrt initial robot frame
  return posR;

}



void OdomPublisher::work(){

    string odom_name = "/odom"+this->robot;
    cout << "Robot odometry " << this->robot <<" -->" << odom_name <<  endl;


    //publishers for positions wrt initial robot frames
    ros::Publisher pos_odom = posPub.advertise<geometry_msgs::Pose>((char*) odom_name.c_str(),100);

    //subscribers for positions wrt world frame
    string robot_odom_name = "/robot"+(this->robot);
    robot_odom_name = robot_odom_name+"/odom";
    cout << "Robot subscribers " << this->robot <<" -->" << robot_odom_name <<  endl;


    ros::Subscriber calc_sub = poseSub.subscribe((char*) robot_odom_name.c_str(),1000,&OdomPublisher::gazeboOdomCallback,this);
    ros::Rate loop_rate(10);

    while (ros::ok()){

//        //check whether each initial robot frame is saved
          flag = 1;
//        for(int i=0;i<N;i++){
//            flag = flag * flagN[i];
//        }



        if(flag==1){//if each initial robot frame is saved, calculates positions wrt initial robot frames

            //positions wrt initial robot frames
            vector<double> odom = calculatePos( );

            //geometry_msgs::Pose messages for each robot using previous calculations
            geometry_msgs::Pose odomPos;
            odomPos.position.x = odom[0]; odomPos.position.y = odom[1];  odomPos.position.z = odom[2];


            //publishes each position
            pos_odom.publish(odomPos);


        }
        else{

            ROS_INFO("waiting...");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

