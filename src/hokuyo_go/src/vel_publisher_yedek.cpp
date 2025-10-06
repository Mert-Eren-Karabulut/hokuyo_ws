//This code is for moving the motors of the pan tilt mechanism.
#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <iostream>
#include <fstream>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <chrono>


#define PI 3.14159265359

double tiltangle=0.0;
double tiltangle_goal = 0.5; // Change to the desired tilt angle - 10 degree considering the error
double panangle=0.0;
double panangle_goal = 0.5;      // Change to the desired pan angle
double angle_vel=0.0;
double panangle_vel=0.0;
double goal_reached_angle=0;
double goal_reached_panangle=0;
double tolerance=0.003; //normal value for reaching the desired angle (0.01)
double max_vel_angle=1*PI/180;
double max_vel_panangle=1*PI/180;
double k=1;
void compare(){

    // for tilt target angle comparison
    if(abs(tiltangle-tiltangle_goal)<tolerance){
    goal_reached_angle=1;
    ROS_INFO("tilt goal reached");
    }

    //for pan target angle comparison
    if(abs(panangle-panangle_goal)<tolerance){
    goal_reached_panangle=1;
    //ROS_INFO("pan goal reached");
    }
}


//gets the current position and calculates the velocities
void encoCallback(const sensor_msgs::JointState::ConstPtr& msg){

    //getting angle and panangle info
    tiltangle = msg->position[1];
    panangle= msg->position[0];
    
    //check if goal is reached
    compare();
    
    //set next goals
    if(goal_reached_angle==1){
        //angle_vel = 0.0; //stop tilting
        tiltangle_goal= -tiltangle_goal; //normalde 0 dı başa dönmesi için
        goal_reached_angle= 0;
    }
    else{
        angle_vel = k * (tiltangle_goal - tiltangle);
        if(fabs(angle_vel) > max_vel_angle){
            angle_vel = max_vel_angle * angle_vel / fabs(angle_vel);
        }
    }
    
    if(goal_reached_panangle==1){
        //panangle_vel = 0.0; //stop panning
        panangle_goal= -panangle_goal;
        goal_reached_panangle= 0;
    } 
    else{
        panangle_vel = k * (panangle_goal - panangle);
        if(fabs(panangle_vel) > max_vel_panangle){
            panangle_vel = max_vel_panangle * panangle_vel / fabs(panangle_vel);
        }
    }
    
    
    //calculating velocities
    // angle_vel=50*(angle_goal-angle); //coeff. 15
    // panangle_vel=50*(panangle_goal-panangle);

    //print angle_vel and panangle_vel
    ROS_INFO("Angle vel: %.6f", angle_vel);
    //ROS_INFO("Pan Angle vel: %.6f", panangle_vel);

//     if(abs(angle_vel)>max_vel_angle){
//         angle_vel=max_vel_angle*angle_vel/abs(angle_vel);
//     }
//     if(abs(panangle_vel)>max_vel_panangle){
//         panangle_vel=max_vel_panangle*panangle_vel/abs(panangle_vel);
//     }
}

int main(int argc,char** argv){

    using Clock = std::chrono::high_resolution_clock;
    // Start measuring time
    auto start = Clock::now();
    // Perform some operations or wait for a certain event
    std::cout << "Measuring time...\n";
    

    ros::init(argc,argv,"vel_publisher_yedek");
    ros::NodeHandle nh;
    
    //subscribing topics
    ros::Subscriber sub = nh.subscribe("/joint_states", 1000, encoCallback);
    
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
        // if(angle >= 0.45){
        //     using Clock = std::chrono::high_resolution_clock;
        //     // Start measuring time
        //     auto start = Clock::now();
        //     // Perform some operations or wait for a certain event
        //     std::cout << "Measuring time...\n";
        //     // Stop measuring time
        // }
        // if(angle >= 0.65){    
        //     auto end = Clock::now();
        //     std::cout << "Measuring time STOPPED\n";
        //     std::chrono::duration<double> elapsed_seconds = end - start;
        //     std::cout << "Time measured: " << elapsed_seconds.count() << "s\n";
        //     double angle_highres_vel = 0.20 / elapsed_seconds.count();
        //     std::cout << "High resolution tilt velocity: " << angle_highres_vel << " rad/s\n";
        // }
        joint_traj.header.stamp = ros::Time::now();
        traj_point.time_from_start = ros::Duration(2.0); // 2 seconds
        traj_point.positions = {panangle_goal, tiltangle_goal};
        traj_point.velocities = {panangle_vel, angle_vel};
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
