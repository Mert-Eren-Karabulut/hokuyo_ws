//This code is for moving the motors of the pan tilt mechanism.
#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <fstream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


#define PI 3.14159265359

ros::Subscriber sub; //declaring sub as a global variable

double tiltangle=0.0;
double tiltangle_goal = 0.4; // Change to the desired tilt angle - 10 degree considering the error
double panangle=0.0;
double panangle_goal = 0.3;      // Change to the desired pan angle
double vel_tilt=0.0;
double vel_pan=0.0;
double goal_reached_angle=0;
double goal_reached_panangle=0;
double tolerance=0.0001; //normal value for reaching the desired angle (0.01)
double max_vel_angle=0*PI/180;
double max_vel_panangle=1*PI/180;
double k=1;
float vel_ref[] = {0.2, 0.15}; // min values are 0.2 and 0.15 respectively
int state_1 = 0;
int state_2 = 0;
int save = 0;




// void compare(){
//     // ROS_INFO("Angle: %.6f", angle);
//     // ROS_INFO("Angle goal: %.6f", angle_goal);
//     if(abs(angle-angle_goal)<tolerance){
//     goal_reached_angle=1;
//     //ROS_INFO("tilt goal_reached");
    
//     //ROS_INFO("Angle vel: %.6f", angle_vel);

//     }
//     if(abs(panangle-panangle_goal)<tolerance){
//     goal_reached_panangle=1;

//     //ROS_INFO("pan goal_reached");   //for pan motion activate this part
//     //ROS_INFO("Pan Angle: %.6f", panangle);
//     //ROS_INFO("Pan Angle goal: %.6f", panangle_goal);
//     //ROS_INFO("Pan Angle vel: %.6f", panangle_vel);
//     }
// }

//gets the current position and calculates the velocities
void encoCallback(const sensor_msgs::JointState::ConstPtr& msg){
    //getting angle and panangle info
    tiltangle = msg->position[1];
    panangle= msg->position[0];
    
    
    // if (state_1 == 0 && abs(angle) < 0.1 && state_2 == 0 && abs(panangle) < 0.1)
    // {
    //     //vel_tilt = -vel_ref[0];  //hıza eksi koyunca açı pozitif tarafa hareket ediyor 
    //     vel_pan = vel_ref[1];
    //     ROS_INFO("Angle: %.6f", angle);
    //     ROS_INFO("PanAngle: %.6f", panangle);
    //     ROS_INFO("vel_tilt: %.6f", vel_tilt);
    //     ROS_INFO("vel_pan: %.6f", vel_pan);
    //     ROS_INFO("First tilt and pan movement is working");
    //     state_1++;
    //     state_2++;
    // }

    
    // if (angle >= angle_goal && state_1 == 1)
    // {
    //     vel_tilt = vel_ref[0];
    //     //print vel_tilt    
    //     ROS_INFO("vel_tilt: %.6f", vel_tilt);
    //     ROS_INFO("Angle: %.6f", angle);
    //     ROS_INFO("Second tilt movement is working");
    //     ROS_INFO("state 1 : %d", state_1);
    //     //print angle
    //     state_1++;
    // }

    // if ( panangle <= -panangle_goal && state_2 == 1) //panangle > 0.3 && state_2 == 1
    // {
    //     //vel_tilt = vel_ref[0];
    //     vel_pan = 0.0;
    //     //print vel_pan
    //     ROS_INFO("vel_pan: %.6f", vel_pan);
    //     ROS_INFO("PanAngle: %.6f", panangle);
    //     //ROS_INFO("Second pan movement is working");
    //     ROS_INFO("state 2 : %d", state_2);
    //     state_2++;
    // }

    // if (angle <= -angle_goal && state_1 == 2)
    // {
    //     vel_tilt = -vel_ref[0];
    //     //print vel_tilt
    //     ROS_INFO("vel_tilt: %.6f", vel_tilt);
    //     ROS_INFO("Angle: %.6f", angle);
    //     ROS_INFO("Third tilt movement is working");
    //     ROS_INFO("state 1 : %d", state_1);
    //     state_1++;

    // }

    // if (panangle >= panangle_goal && state_2 == 2) // panangle < -0.3 && state_2 == 2
    // {
    //     vel_pan = vel_ref[1];
    //     //print vel_pan
    //     ROS_INFO("vel_pan: %.6f", vel_pan);
    //     ROS_INFO("PanAngle: %.6f", panangle);
    //     ROS_INFO("Third pan movement is working");
    //     ROS_INFO("state 2 : %d", state_2);
    //     state_2++;

    // }

    // if (angle >= angle_goal && state_1 ==3)
    // {
    //     vel_tilt = vel_ref[0];
    //     //print vel_tilt
    //     ROS_INFO("vel_tilt: %.6f", vel_tilt);
    //     ROS_INFO("Angle: %.6f", angle);
    //     ROS_INFO("Fourth tilt movement is working");
    //     ROS_INFO("state 1 : %d", state_1);
    //     state_1++;
        
    // }

    // if (panangle <= -panangle_goal && state_2 ==3) // panangle > 0.3 && state_2 ==3
    // {
    //     vel_pan = -vel_ref[1];
    //     ROS_INFO("PanAngle: %.6f", panangle);
    //     //print vel_pan
    //     ROS_INFO("vel_pan: %.6f", vel_pan);
    //     ROS_INFO("Fourth pan movement is working");
    //     ROS_INFO("state 2 : %d", state_2);
    //     state_2++;
    // }

    // if (angle <= 0.0  && state_1 == 4)
    // {
    //     vel_tilt = 0.0;        
    //     state_1++;
    //     ROS_INFO("Tilt movement stopped");
    //     ROS_INFO("state 1 : %d", state_1);
    // }

    // if (state_2 == 4 && panangle >= 0.0)
    // {
    //     vel_pan = 0.0;
    //     ROS_INFO("Pan movement stopped");
    //     ROS_INFO("state 2 : %d", state_2);
    //     state_2++;
    //     //std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    // }

    // if (state_1 == 5) //state_1 == 5 && state_2 == 5
    // {
    //     save = 1;
    //     ROS_INFO("Now the saving will be done");
    //     state_1++;
    //     state_2++; //ikisi de 6 oldu
    // }
    
    

}
int main(int argc,char** argv){

    ros::init(argc,argv,"vel_publisher");
    ros::NodeHandle nh;
    
    //subscribing topics
    sub = nh.subscribe("/joint_states", 1000, encoCallback);
    
    //publisher for velocity at gazebo
    ros::Publisher vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/gazebo_ros_control/command", 1000);
    
    trajectory_msgs::JointTrajectory joint_traj;
    trajectory_msgs::JointTrajectoryPoint traj_point;
    joint_traj.joint_names = {"pan_joint", "tilt_joint"};
    
    
    // Create a joint trajectory point
    //traj_point.time_from_start = ros::Duration(2.0); // 2 seconds
    //traj_point.positions = {1.0, 2.0};

    // Add the joint trajectory point to the joint trajectory message
    //joint_traj.points.push_back(traj_point);
    
    
    ros::Rate loop(50);
    while(ros::ok()){
      joint_traj.header.stamp = ros::Time::now();
      traj_point.time_from_start = ros::Duration(15.0); // 2 seconds
      traj_point.positions = {0.0, 0.0}; // Change to the desired pan and tilt angles
      traj_point.velocities = {vel_pan, vel_tilt}; 


      //traj_point.velocities.push_back(0.5);  // Velocity for pan_joint
      //traj_point.velocities.push_back(0.3);  // Velocity for tilt_joint
      //joint_traj.points.positions= {angle_goal, panangle_goal};
      //ROS_INFO("pos sizes; %ld vel size: %ld", traj_point.positions.size(),traj_point.velocities.size());
      joint_traj.points.clear();
      joint_traj.points.push_back(traj_point);

      vel_pub.publish(joint_traj);

      ros::spinOnce();
      loop.sleep();

    }
}

// void dummy(){
//     geometry_msgs::Point angles;
//     angles.x = 0.0;
//     angles.y = 0.0;

//     ros::Publisher pubAngles = nh.advertise<geometry_msgs::Point>("/target_head_apf", 1);

//     pubAngles.publish(angles);

// }