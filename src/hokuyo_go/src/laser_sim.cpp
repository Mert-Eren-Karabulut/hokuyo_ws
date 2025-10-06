/*
this code is revised and improved to get 
pointcloud data from laser scanner for 
certain number of wanted scan number of the laser sensor.
It gives array of length 640 for each scan and pointcloud data is calculated 
with the formula given below. It depends on the scan_number that you give.
*/

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/JointState.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <vector>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "utility.h"

Conncomp1d conncomp;
StdDeviation stddev;

#define PI 3.14159265359

using namespace std;

ros::Subscriber laserSub;

std::vector<float> laser_scan;
std::vector<std::vector<float>> laser_matrix;

ros::Time previous_time;
double previous_angle = 0.0;
double previous_panangle = 0.0;
double tiltangle = 0.0;
double panangle = 0.0;
double laser_increment = 0.0;
double panangle_goal = 0.0;
double tiltangle_goal = 0.0;
double lastangle = 0.0;
double lastpan = 0.0;
double vel_pan = 0.0;
double vel_tilt = 0.0;
float vel_ref[] = {0.5, 0.5}; // min values are 0.05 and 0.03 respectively
double tolerance = 0.01; //normal value for reaching the desired angle (0.01)
double tolerance_pan = 0.001;
double max_vel_tilt=1*PI/180;
double max_vel_pan=1*PI/180;
int goal_reached_tiltangle = 0;
int goal_reached_panangle = 0;
int k = 1;

float max_center_value;
int max_center_index = -1;
int max_component_width = 0.0;
int start_index = -1;
int end_index = -1;
int component_width;
double component_centroid;
float center_comp_angle = 0.0;

int centroid_index;

int state = 0;

std::vector<float> center_values;

//for pointcloud conversion
float x[1024];
float y[1024];
float z[1024];
int offset;
int ctn = 0;
int t =0;
int scanSize;
int a=1;
int b=0;
int d1=0.05;
int d2=0.1;
float d_1=0.114;
float d_2=0.02845;
float l_1=0.045;
float l_2=0.040; //0.075
float xx;
float yy;
float zz;
float e_x,e_y,e_z;
float start_angle;
int conncomp_state = 0;
int kinematic_state = 0;

int scan_num = 900; //how many scans do we want to get

//declaring the lasercan and pointcloud objects
sensor_msgs::LaserScan last_laser;
sensor_msgs::PointCloud2 cloud_out;

// Laser data callback
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //assigning laser data
    laser_increment = msg->angle_increment;
    laser_scan = msg->ranges;

    std::vector<float> row(laser_scan.begin(), laser_scan.end());
    laser_matrix.push_back(row);

    int laser_count = laser_scan.size();

    std::vector<float> laser_values(laser_scan.begin(), laser_scan.end());
    int binaryImage[laser_count];

    stddev.SetValues(laser_values, laser_count); // Set laser data for standard deviation calculation
    double standard_deviation = stddev.GetStandardDeviation(); // Calculate standard deviation
    double threshold = standard_deviation/2 ; // Set threshold as half the standard deviation

    //set binay image first using only detected image ranges
    for (int i = 0; i < laser_count; ++i) {
        if (laser_scan[i] < 6.0) {
            binaryImage[i] = 1; // Object detected within the threshold
        } else {
            binaryImage[i] = 0; // No object detected or beyond threshold
        }
    }

    //change the binary image values using threshold between two consecutive laser data
    for (int i = 1; i<laser_count; ++i) {
        if (fabs(laser_scan[i] - laser_scan[i-1]) > threshold) {
            //ROS_INFO("Difference between two consecutive laser data is beyond threshold");
            binaryImage[i] = 0; // Set as 0 if the difference between two consecutive laser data is beyond threshold
        }
    }

    conncomp.SetValues1(binaryImage, laser_count);

    //determine the number of connected components
    int num_components = conncomp.findcompNumber();
    //process connected component width and centroid
    conncomp.findcompwidth();
    conncomp.findcentroid();

    //access component information
    for (int i = 0; i < num_components; ++i) {
        component_width = conncomp.compwidth[i];
        component_centroid = conncomp.centroid[i];

        //process or store component width and centroid as needed
        //ROS_INFO("Component %d: Width = %.6f, Centroid = %.6f", i+1, component_width, component_centroid);
        
        //get the nearest integer to component_centroid 
        centroid_index = (int)component_centroid;
        //ROS_INFO("laser value at the center of the component: %.6f", laser_scan[centroid_index]);
        //ROS_INFO("centroid index: %d", centroid_index);
        //form an array for center values of the cmponents
        center_values.push_back(laser_scan[centroid_index]);
        
        //compute the biggest value in the center values
        max_center_value = *std::max_element(center_values.begin(), center_values.end());

        // Determine the index of the maximum center value
        if(laser_scan[centroid_index] == max_center_value){
            max_center_index = centroid_index;
            max_component_width = component_width;
        }

        if(centroid_index==max_center_index){
            //first component
            start_index = max_center_index - component_width/2;
            end_index = max_center_index + component_width/2;
        }
    }
    center_comp_angle = max_center_index*laser_increment;

    if(conncomp_state == 0){
        ROS_INFO("Number of connected components: %d", num_components); // Print the number of connected components
        ROS_INFO("Farthest component's center laser value: %.6f meters", max_center_value);
        ROS_INFO("Farthest component's center laser angle value: %.6f radians", center_comp_angle);
        //ROS_INFO("Start index of the farthest component: %d", start_index);
        //ROS_INFO("End index of the farthest component: %d", end_index);
        //ROS_INFO("Farthest component's starting angle: %.6f radians", start_index * laser_increment);
        //ROS_INFO("Farthest component's ending angle: %.6f radians", end_index * laser_increment);
        conncomp_state++;
    }
        
    

    //use center_comp_angle for determining the speed of the robot

    last_laser = *msg;
    //ctn++;

    // if (ctn == scan_num)
    // {
    //     ROS_INFO("Received all laser scans. Shutting down laser subscriber.");
    //     //laserSub.shutdown();
    // }
}

//to save the laser scans to a file
void saveLaserMatrixtoFile(const std::vector<std::vector<float>>& matrix, const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error: Could not open file " << filename << " for writing" << std::endl;
        return;
    }
    //Write each row of the matrix to the file
    for (const auto& row : matrix)
    {
        for (size_t i = 0; i<row.size(); ++i)
        {
            file << row[i] << " ";
            if (i != row.size()-1)
            {
                file << ", ";
            }
        }
        file << std::endl;
    }
    file.close();
    std::cout << "Laser matrix saved to file " << filename << std::endl;
}

void compare(){

    // for tilt target angle comparison
    if(abs(tiltangle-tiltangle_goal)<tolerance){
    goal_reached_tiltangle=1;
    //ROS_INFO("tilt goal_reached");
    }

    //for pan target angle comparison
    if(abs(panangle-panangle_goal)<tolerance){
    goal_reached_panangle=1;
    }
}

// void moveTilt(double anglegoaltilt) {
//     int state_tilt = 0;
//     if (state_tilt == 0) {
//         tiltangle_goal = anglegoaltilt;
//         kinematic_state = 1;
//         state_tilt++;
//     }

//     if(state_tilt == 1 && goal_reached_tiltangle == 1){
//         tiltangle_goal = -anglegoaltilt;
//         kinematic_state = 0;
//         state_tilt++;
//     }

//     if(state_tilt == 2 && goal_reached_tiltangle == 1){
//         tiltangle_goal = 0.0;
//         kinematic_state = 1;
//         state_tilt++;
//     }

//     if(state_tilt == 3 && goal_reached_tiltangle == 1){
//         ROS_INFO("TILT MOVEMENT COMPLETED");
//     }
// }

// void movePan(double anglegoalpan) {
//     int state_pan = 0;
//     if (state_pan == 0) {
//         panangle_goal = anglegoalpan;
//         kinematic_state = 1;
//         state_pan++;
//     }

// }

//joint states callback function
void encoCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
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
        
        state++;
    } 
    
    if (state == 1 && goal_reached_tiltangle == 1){
        tiltangle_goal = 0.0;
        kinematic_state = 1;
        goal_reached_tiltangle = 0;
        state++;
        ROS_INFO("TILT MOVEMENT COMPLETED");
    }

    if (state == 2 && panangle <= tolerance && goal_reached_tiltangle == 1){
        ROS_INFO("PAN MOVEMENT STARTED");
        kinematic_state = 1;
        panangle_goal = -0.0214755783; //-1.051147578;
        
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
        ROS_INFO("TILT MOVEMENT COMPLETED"); 
    }

    if (state == 6 && goal_reached_tiltangle == 1) {
        
        panangle_goal = 0.0214755783; //this is the final pan movement for 1 pan mpvement
        
        state++;
    }

    if (state == 7 && fabs(panangle-panangle_goal) <= tolerance_pan){
        //ROS_INFO("PAN MOVEMENT COMPLETED");
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

    if(state == 6 && goal_reached_panangle == 1 && goal_reached_tiltangle == 1){
        ROS_INFO("MOVEMENT COMPLETED");
        
        state++;
    }

    //ROS_INFO("Goal reached panangle: %d", goal_reached_panangle);

    
    

    //print angle_vel and panangle_vel
    //ROS_INFO("Angle vel: %.6f", angle_vel);
    //ROS_INFO("Pan Angle vel: %.6f", panangle_vel);















    //now pcl conversion with kinematics

    if ( kinematic_state == 0) {
        //ROS_INFO("Processing point cloud data...");
        if(ctn<scan_num){
            
            scanSize= (int)laser_scan.size();
            
            //assigning point cloud attributes
            cloud_out.height = 1;
            cloud_out.width  = scanSize*scan_num;
            cloud_out.fields.resize (3);
            cloud_out.fields[0].name = "x";
            cloud_out.fields[0].offset = 0;
            cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            cloud_out.fields[0].count = 1;
            cloud_out.fields[1].name = "y";
            cloud_out.fields[1].offset = 4; //4
            cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            cloud_out.fields[1].count = 1;
            cloud_out.fields[2].name = "z";
            cloud_out.fields[2].offset = 8; //8
            cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            cloud_out.fields[2].count = 1;

            int offset = 12;

            cloud_out.point_step = offset;
            cloud_out.row_step   = cloud_out.point_step * cloud_out.width;
            cloud_out.data.resize (cloud_out.row_step   * cloud_out.height);
            cloud_out.is_dense = false;

            unsigned int count = ctn*scanSize;

// 
            for(int i=0; i<scanSize; i++){

           
                start_angle=-2.094;
                yy= laser_scan[i]*sin(laser_increment*i+start_angle);
                xx= laser_scan[i]*cos(laser_increment*i+start_angle);
                zz= 0;
                // simülasyon için değiştirilecek 
                x[i]=xx*cos(panangle)-
                                    yy*sin(panangle)*cos(tiltangle)+zz*sin(panangle)*sin(tiltangle)+
                                    l_1*sin(panangle)*sin(tiltangle)+l_2*sin(panangle);
                                    
                y[i]=xx*sin(panangle)+
                                    yy*cos(panangle)*cos(tiltangle)-zz*cos(panangle)*sin(tiltangle)-
                                    l_1*cos(panangle)*sin(tiltangle)+l_2*cos(panangle)-d_2;
                z[i]=yy*sin(tiltangle)+zz*cos(tiltangle)+l_1*cos(tiltangle)+d_1;
                // değiştirme
                
                // ROS_INFO("xx: %.6f", xx);
                // ROS_INFO("yy: %.6f", yy);
                //ROS_INFO("zz: %.6f", zz);
                // ROS_INFO("x: %.6f", x[i]);
                // ROS_INFO("y: %.6f", y[i]);
                // ROS_INFO("z: %.6f", z[i]);
                // ROS_INFO("i: %d", i);
                


                float *pstep = (float*)&cloud_out.data[count * cloud_out.point_step];

                pstep[0] = x[i];  //pointclouda ekleme
                pstep[1] = y[i];
                pstep[2] = z[i];
                ++count;

            

            }
            
            ctn++;
            

        }
        
        //reseting data at certain scan number
        else{
        //cloud_out.data.clear();
        //ctn=0;
        }
        

        //ctn++;
        lastangle=tiltangle;
    	lastpan=panangle;
    }


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_sim");
    ros::NodeHandle n;

    //subscribing to the laser scan data
    laserSub = n.subscribe<sensor_msgs::LaserScan>("/hokuyo/scan", 1000, laserCallback);
    // joint states subscriber
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, encoCallback);

    //publisher for pointcloud data
    ros::Publisher pclPub = n.advertise<sensor_msgs::PointCloud2>("/output", 10);
    
    //velocity publisher
    ros::Publisher vel_pub = n.advertise<trajectory_msgs::JointTrajectory>("/gazebo_ros_control/command", 1000);
    trajectory_msgs::JointTrajectory joint_traj;
    trajectory_msgs::JointTrajectoryPoint traj_point;
    joint_traj.joint_names = {"pan_joint", "tilt_joint"};

    ros::Rate loop(50);
    while(ros::ok()){

        joint_traj.header.stamp = ros::Time::now();
        traj_point.time_from_start = ros::Duration(4.0); // 15 seconds //this represents when the robot be in this position
        traj_point.positions = {panangle_goal, tiltangle_goal}; // Change to the last desired pan and tilt angles for robot to stay in that position
        traj_point.velocities = {vel_pan, vel_tilt}; 

        cloud_out.header.frame_id = "taban";
        cloud_out.header.stamp = last_laser.header.stamp;
        cloud_out.header.seq = cloud_out.header.seq+1;

        joint_traj.points.clear();
        joint_traj.points.push_back(traj_point);

        vel_pub.publish(joint_traj);
        pclPub.publish(cloud_out);

        ros::spinOnce();
        loop.sleep();

        // if(ctn == scan_num)
        // {
        //     //sub.shutdown();
        //     //laserSub.shutdown();
        //     //pclPub.shutdown();
        //     //vel_pub.shutdown();
        //     //saveLaserMatrixtoFile(laser_matrix, "laser_matrix.csv");
        //     ROS_INFO("Pointcloud data processing completed.");
        //     break;
        // }
    }

    return 0;

}