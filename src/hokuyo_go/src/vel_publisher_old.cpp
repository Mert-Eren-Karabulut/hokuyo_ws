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
double tiltangle_goal = 0.7; // Change to the desired tilt angle - 10 degree considering the error
double panangle=0.0;
double panangle_goal = 0.0;      // Change to the desired pan angle
double angle_vel=0.0;
double tiltangle_start = 0.0;
double panangle_vel=0.0;
int goal_reached_tiltangle=0;
int goal_reached_panangle=0;
double tolerance=0.08; //normal value for reaching the desired angle (0.01)
double tolerance_pan = 0.001;
double max_vel_angle=0.3; //1*PI/180;
double max_vel_panangle=1*PI/180;
double k=-1;
int state = 0;
double previous_angle = 0.0;
double previous_panangle = 0.0;
ros::Time previous_time;
int kinematic_state = 0;
double elapsed_time = 0.0;

using Clock = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;

TimePoint start_time;

void compare(){

    // for tilt target angle comparison
    if(abs(tiltangle-tiltangle_goal)<tolerance){
    goal_reached_tiltangle=1;
    //ROS_INFO("tilt goal reached");
    }

    //for pan target angle comparison
    if(abs(panangle-panangle_goal)<tolerance){
    goal_reached_panangle=1;
    //ROS_INFO("pan goal reached");
    }
}

void startTimeMeasurement() {
    start_time = Clock::now();
}

double stopTimeMeasurement() {
    TimePoint end_time = Clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    return elapsed_seconds.count();
}


//gets the current position and calculates the velocities
void encoCallback(const sensor_msgs::JointState::ConstPtr& msg){

    // if (previous_time.isZero()) {
    //     previous_time = msg->header.stamp;
    //     previous_angle = msg->position[1];  // Tilt joint
    //     previous_panangle = msg->position[0];  // Pan joint
    //     return;
    // }

    // ros::Time current_time = msg->header.stamp;

    //getting angle and panangle info
    tiltangle = msg->position[1];
    panangle= msg->position[0];

    // Calculate time difference
    // ros::Duration dt = current_time - previous_time;

    // // Calculate joint angle velocities (radians per second)
    // double angle_vel = (angle - previous_angle) / dt.toSec();
    // double panangle_vel = (panangle - previous_panangle) / dt.toSec();


    //print the previous time and current time 
    // ROS_INFO("Previous time: %f", previous_time.toSec());
    // ROS_INFO("Current time: %f", current_time.toSec());

    // // Print joint velocities for debugging
    // ROS_INFO("Tilt velocity: %.6f rad/s", angle_vel);
    // ROS_INFO("Pan velocity: %.6f rad/s", panangle_vel);

    // Update previous values for the next callback
    // previous_time = current_time;
    // previous_angle = angle;
    // previous_panangle = panangle;

    //check if goal is reached
    compare();
    
    if (state == 0 && tiltangle <= tolerance && panangle <= tolerance){
        tiltangle_goal = 0.5235987756;
        kinematic_state = 1;
    }

    if (state == 0 && fabs(tiltangle-tiltangle_goal) <= tolerance) {
        tiltangle_goal = -tiltangle_goal;
        kinematic_state = 0;
        goal_reached_tiltangle = 0;
        startTimeMeasurement();
        tiltangle_start = tiltangle;

        state++;
    } 
    
    if (state == 1 && goal_reached_tiltangle == 1){
        tiltangle_goal = 0.0;
        kinematic_state = 1;
        goal_reached_tiltangle = 0;
        state++;
        elapsed_time = stopTimeMeasurement();
        ROS_INFO("Measured tilt velocity for resolution: %.6f rad/s", fabs(tiltangle - tiltangle_start) / elapsed_time);

        ROS_INFO("TILT MOVEMENT COMPLETED");
    }

    if (state == 2 && panangle <= tolerance && goal_reached_tiltangle == 1){
        ROS_INFO("PAN MOVEMENT STARTED");
        kinematic_state = 1;
        panangle_goal = -1.051147578;
        
        state++;
    }

    if (state == 3 && fabs(panangle-panangle_goal) <= tolerance_pan){
        ROS_INFO("PAN MOVEMENT COMPLETED");
        goal_reached_panangle = 0;
        kinematic_state = 1;
        tiltangle_goal = 0.5235987756;
        
        state++;
    }

    if (state == 4 && fabs(tiltangle-tiltangle_goal) <= tolerance) {
        tiltangle_goal = -tiltangle_goal;
        kinematic_state = 0;
        goal_reached_tiltangle = 0;
        state++;
    }

    if (state == 5 && goal_reached_tiltangle == 1){
        tiltangle_goal = 0.0;
        kinematic_state = 1;
        goal_reached_tiltangle = 0;
        state++;
        //ROS_INFO("TILT MOVEMENT COMPLETED"); 
    }

    if (state == 6 && goal_reached_tiltangle == 1) {
        
        panangle_goal = 1.051147578; //this is the final pan movement for 1 pan mpvement
        
        state++;
    }

    if (state == 7 && fabs(panangle-panangle_goal) <= tolerance_pan){
        ROS_INFO("PAN MOVEMENT COMPLETED");
        goal_reached_panangle = 0;
        kinematic_state = 1;
        tiltangle_goal = 0.5235987756;
        
        state++;
    }

    if (state == 8 && fabs(tiltangle-tiltangle_goal) <= tolerance) {
        tiltangle_goal = -tiltangle_goal;
        kinematic_state = 0;
        goal_reached_tiltangle = 0;
        state++;
    }

    if (state == 9 && goal_reached_tiltangle == 1){
        tiltangle_goal = 0.0;
        kinematic_state = 1;
        goal_reached_tiltangle = 0;
        state++;
        ROS_INFO("TILT MOVEMENT COMPLETED");
    }

    if (state == 10 && goal_reached_tiltangle == 1) {
        
        panangle_goal = 0.0; 
        
        state++;
    }

    if(state == 11 && goal_reached_panangle == 1 && goal_reached_tiltangle == 1){
        ROS_INFO("MOVEMENT COMPLETED");
        
        state++;
    }

    //ROS_INFO("Goal reached panangle: %d", goal_reached_panangle);

    
    

    //print angle_vel and panangle_vel
    //ROS_INFO("Angle vel: %.6f", angle_vel);
    //ROS_INFO("Pan Angle vel: %.6f", panangle_vel);

}

int main(int argc,char** argv){

    
    ros::init(argc,argv,"vel_publisher");
    ros::NodeHandle nh;
    
    //subscribing topics
    ros::Subscriber sub = nh.subscribe("/joint_states", 300, encoCallback);
    
    //publisher for velocity at gazebo
    ros::Publisher vel_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/gazebo_ros_control/command", 1000);
    
    trajectory_msgs::JointTrajectory joint_traj;
    trajectory_msgs::JointTrajectoryPoint traj_point;
    joint_traj.joint_names = {"pan_joint", "tilt_joint"};
    
    
    ros::Rate loop(50);
    while(ros::ok()){


        // if(angle >= 0.45){
        //     using Clock = std::chrono::high_resolution_clock;
        //     auto start = Clock::now();
        //     // Perform some operations or wait for a certain event
        //     std::cout << "Measuring time...\n";
        //     // Stop measuring time
        //     if(angle >= 0.65){    
        //         auto end = Clock::now();
        //         std::cout << "Measuring time STOPPED\n";
        //         std::chrono::duration<double> elapsed_seconds = end - start;
        //         std::cout << "Time measured: " << elapsed_seconds.count() << "s\n";
        //         double angle_highres_vel = 0.20 / elapsed_seconds.count();
        //         std::cout << "High resolution tilt velocity: " << angle_highres_vel << " rad/s\n";
        //     }
        // }
        
        
        joint_traj.header.stamp = ros::Time::now();
        traj_point.time_from_start = ros::Duration(4.0); // 2 seconds
        traj_point.positions = {panangle_goal, tiltangle_goal};
        //traj_point.velocities = {panangle_vel, angle_vel};
        

        joint_traj.points.clear();
        joint_traj.points.push_back(traj_point);

        vel_pub.publish(joint_traj);

        ros::spinOnce();
        loop.sleep();

    }
}
