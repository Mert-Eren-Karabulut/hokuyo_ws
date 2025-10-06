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
#include <vector>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>


#define PI 3.14159265359

using namespace std;

ros::Subscriber laserSub;

std::vector<float> laser_scan;
std::vector<std::vector<float>> laser_matrix;

double tiltangle = 0.0;
double panangle = 0.0;
double lastangle = 0.0;
double lastangle2 = 0.0;
double lastpan=0;
double laser_increment = 0.0;
// double tilt_min = 0.0;
// double tilt_max = 0.2;
double panangle_goal = 0.0460190964;
double tiltangle_goal = 0.0;
double vel_pan = 0.0;
double vel_tilt = 0.0;
float vel_ref[] = {0.05, 0.03}; // min values are 0.2 and 0.15 respectively

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
int state_1 = 0;
int state_2 = 0;

float d_1=0.114;
float d_2=0.02845;
float l_1=0.045;
float l_2=0.040; //0.075
float xx;
float yy;
float zz;
float e_x,e_y,e_z;
float start_angle;

int scan_num=500; //how many scans do we want to get



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

    last_laser = *msg;
    ctn++;

    if (ctn == scan_num)
    {
        ROS_INFO("Received all laser scans. Shutting down laser subscriber.");
        laserSub.shutdown();
    }

}
//to visualize the matrix with laser scans shape scan_num x 726
void visualizeLasermatrix()
{
    ROS_INFO("Number of rows in matrix: %zu", laser_matrix.size());

    if (!laser_matrix.empty())
    {
        ROS_INFO("Number of columns in matrix: %zu", laser_matrix[0].size());
    }

    for (size_t i = 0; i<laser_matrix.size(); i++) {
        //ROS_INFO("Row %zu: ", i);
        for (size_t j = 0; j<laser_matrix[i].size(); j++) {
            //ROS_INFO("Row %zu: and Column %zu: %f", i, j, laser_matrix[i][j]);
        }
    }
}

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
        

// Joint states callback
void encoCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //getting joint state data


    tiltangle = msg->position[1];
    panangle= msg->position[0];

    //begin the movement when the robot is in the initial position with tilt movement

    if (state_1 == 0 && abs(tiltangle) < 0.1 && state_2 == 0 && abs(panangle) < 0.1)
    {
        vel_tilt = -vel_ref[0]; //hıza eksi koyunca açı pozitif tarafa hareket ediyor
        //ros::Duration(2).sleep();
        //vel_pan = vel_ref[1];
        state_1++;
        ROS_INFO("Tilt angle: %f, Pan angle: %f", tiltangle, panangle);
        ROS_INFO("The movement has started.");
    }

    if (tiltangle >= tiltangle_goal && state_1 == 1)
    {
        vel_tilt = 0.0;
        state_1++;
        ROS_INFO("Tilt angle: %f, Pan angle: %f", tiltangle, panangle);
        ROS_INFO("Second tilt movement is working.");
        
    }

    if (panangle <= -panangle_goal && state_2 == 1)
    {
        vel_pan = 0.0;
        ros::Duration(2).sleep();
        vel_pan = vel_ref[1];
        state_2++;
        ROS_INFO("Tilt angle: %f, Pan angle: %f", tiltangle, panangle);
        ROS_INFO("Second pan movement is working.");
        
        
    }

    if (tiltangle <= -tiltangle_goal && state_1 == 2)
    {
        vel_tilt = vel_ref[0];
        state_1++;
        ROS_INFO("Tilt angle: %f, Pan angle: %f", tiltangle, panangle);
        ROS_INFO("Third tilt movement is working.");
    }

    if (panangle <= -(panangle_goal+3*0.35156+0.08789) && state_2 == 2)
    {
        
        vel_pan = 0.0;
        ros::Duration(2).sleep();
        state_2++;
        ROS_INFO("Tilt angle: %f, Pan angle: %f", tiltangle, panangle);
        ROS_INFO("Third pan movement is working.");
    }

    if (state_1 == 3)
    {
        vel_tilt = 0.0;
        ROS_INFO("Tilt angle: %f, Pan angle: %f", tiltangle, panangle);
        ROS_INFO("Tilt movement is stopped.");
    }

    if (state_2 == 3)
    {
        vel_pan = 0.0;
        ROS_INFO("Tilt angle: %f, Pan angle: %f", tiltangle, panangle);
        ROS_INFO("Pan movement is stopped.");
    }

    //if i durgun halde sensör çıktısı almak için değiştirdik normal hali : if( fabs(angle-lastangle)>=(M_PI)/90 || fabs(panangle-lastpan)>=(M_PI)/720 ){
    
   if( fabs(tiltangle-lastangle)>=(M_PI)/90 || fabs(panangle-lastpan)>=(M_PI)/720 ){
        
        if(ctn<scan_num){
            ROS_INFO("Processing point cloud data...");
            ROS_INFO("panangle is: %f", panangle);
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

            offset = 12;

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
                
                
                float *pstep = (float*)&cloud_out.data[count * cloud_out.point_step];

                pstep[0] = x[i];  //pointclouda ekleme
                pstep[1] = y[i];
                pstep[2] = z[i];
                ++count;

            }
            
            //ctn++;
            // std::vector<float> row(cloud_out.data.begin(), cloud_out.data.end());
            // matrix.push_back(row);
            // ROS_INFO("Matrix: %d", matrix );

        }
        
        //reseting data at certain scan number
        else{
            ROS_INFO("Resetting point cloud data...");
            cloud_out.data.clear();
            //ctn=0;
            //ROS_INFO("ctn is set to 0");
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
  //ROS_INFO("ctn in main: %d", ctn);
 
  //subscribing topics
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, encoCallback);
  laserSub = n.subscribe("/hokuyo/scan", 1000, laserCallback); //hokuyo/scan
  
  //publishing resulting point cloud
  ros::Publisher pclPub = n.advertise<sensor_msgs::PointCloud2> ("output", 10);
  ros::Publisher vel_pub = n.advertise<trajectory_msgs::JointTrajectory>("/gazebo_ros_control/command", 1000);

  //HeadAPF head_apf(n, "/pan_controller/command", "/tilt_controller/command");
  
//   std::vector<float> pan_positions = {0.500, -0.500, 0.000};
//   //std::vector<float> tilt_positions = {0.0, 0.0, 0.0};
//   std::vector<float> durations = {5.0, 15.0, 20.0};
  
  trajectory_msgs::JointTrajectory joint_traj;
  trajectory_msgs::JointTrajectoryPoint traj_point; //in the for loop
  joint_traj.joint_names = {"pan_joint", "tilt_joint"};
//   joint_traj.joint_names = {"pan_joint", "tilt_joint"};
//   joint_traj.points.clear();

//   for (int i=0; i<pan_positions.size(); i++)
//   {
//     trajectory_msgs::JointTrajectoryPoint traj_point;
//     traj_point.time_from_start = ros::Duration(durations[i]);
//     traj_point.positions = {pan_positions[i], 0.0};
//     traj_point.velocities = {vel_pan, vel_tilt};
//     ROS_INFO("State number %d", i);
//     ROS_INFO("Pan angle value %f", panangle);

//     joint_traj.points.push_back(traj_point);
//   }

  //publishing pointcloud

  ros::Rate loop(10);
  while(ros::ok()){
      
      joint_traj.header.stamp = ros::Time::now();
      traj_point.time_from_start = ros::Duration(15.0); // 15 seconds //this represents when the robot be in this position
      traj_point.positions = {0.0, 0.0}; // Change to the last desired pan and tilt angles for robot to stay in that position
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

      if(ctn==scan_num){
        sub.shutdown();
        //laserSub.shutdown();
        pclPub.shutdown();
        vel_pub.shutdown();
        //visualizeLasermatrix();
        //saveLaserMatrixtoFile(laser_matrix, "laser_matrix_wpan0.csv"); //save the laser matrix to a file
        ROS_INFO("Pointcloud data processing completed.");
        
        break;

    }
  }
  return 0;
}
