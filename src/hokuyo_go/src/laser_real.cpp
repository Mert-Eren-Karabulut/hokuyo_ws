// the code for real system movement and saving the pointcloud data
#include "ros/ros.h"                         //ros
#include "std_msgs/Float64.h"                //for float64
#include "std_msgs/Float32.h"                //for float32
#include "sensor_msgs/LaserScan.h"           //for laser
#include "sensor_msgs/PointCloud2.h"         //for pointcloud
#include <sensor_msgs/JointState.h>          //from arduino for encoder
#include <geometry_msgs/Twist.h>             //from arduino for cmd_vel
#include <pcl_conversions/pcl_conversions.h> //for PCL and ROS data type conversion
#include <pcl/point_cloud.h>                 //for pointcloud
#include <pcl/point_types.h>                 //for pointcloud
#include <iostream>                          //input output operations
#include <fstream>                           // for file handling
#include <std_msgs/Bool.h>                   //for boolean
#include <thread>
#include <chrono>
#include <cmath>
#include "utility.h"

using namespace std;
Conncomp1d ccw;
std::vector<float> laser_scan;

double tiltangle = 0.0;
double tiltangle_goal = 0.5;
double panangle_goal = 0.2;
double pan_error = 0.0034973901;
double p = panangle_goal - pan_error;
double tilt_error = 0.0;
double ta = tiltangle_goal - tilt_error;
double tilt = 0.0;
double panangle = 0.0;
double lastangle = 0.0;
double lastangle2 = 0.0;
double lastpan = 0;
double laser_increment = 0.0;
double vel_pan = 0;
double vel_tilt = 0;
int vertical_scan = 0;

float x[1024];
float y[1024];
float z[1024];
int scan_number = 2000; // Number of laser scans to process
int ctn = 0;            // Counter for the processed laser scans
int t = 0;
int scanSize;
int a = 1;
int b = 0;
int d1 = 0.05;
int d2 = 0.1;
float d_1 = 0.114;
float d_2 = 0.02845;
float l_1 = 0.045;
float l_2 = 0.075;
float xx;
float yy;
float zz;
float e_x, e_y, e_z;
float start_angle;
float x1 = 0.106;
float vel_ref[] = {0.1, 0.05}; // 0.05,0.03
int org = 0;
int state_1 = 0;
int state_2 = 0;
double k = 0.3;
int pointcloud_output = 0;
auto start_time = std::chrono::high_resolution_clock::now();
auto end_time = std::chrono::high_resolution_clock::now();
double started_tilt = 0.0;
double measured_veltilt = 0.0;
double vel_state = 0.0;

// declaring the lasercan and pointcloud objects
sensor_msgs::LaserScan last_laser;
sensor_msgs::PointCloud2 cloud_out;
int save = 0;
// Getting laser data

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    laser_increment = msg->angle_increment;
    laser_scan = msg->ranges;
    last_laser = *msg;
}

// Encoder Callback and cmd_vel calculations
void encoCallback(const sensor_msgs::JointState::ConstPtr &msg)
{

    // getting joint states
    tiltangle = msg->position[0];
    panangle = msg->position[1];
    // ROS_INFO("tiltangle: %f", tiltangle);

    // one choice of movement (3-tilt)
    // this movement code needs update, you need to use APF for position control.

    if (state_1 == 0 && state_2 == 0)
    {

        vel_tilt = -k * fabs(tiltangle_goal - tiltangle) / tiltangle_goal;
        if (fabs(vel_tilt) > vel_ref[0])
        {
            vel_tilt = vel_ref[0] * vel_tilt / fabs(vel_tilt);
        }
        ROS_INFO("vel tilt: %f", vel_tilt);
        // vel_tilt = vel_ref[0]; //vel_ref[0];
        // vel_pan = -vel_ref[1];

        // print angle and panangle and org

        // ROS_INFO("state_2: %d", state_2);

        // ROS_INFO("panangle: %f", panangle);
        //  print vel_tilt and vel_pan
        // ROS_INFO("vel_tilt: %f", vel_tilt);
        // ROS_INFO("vel_pan: %f", vel_pan);
        // ROS_INFO("First tilt and pan movement is working");

        if (fabs(tiltangle) >= fabs(tiltangle_goal))
        {
            tiltangle_goal = -tiltangle_goal;
            state_1++;
        }

        // ROS_INFO("angle: %f", tiltangle);
        // ROS_INFO("TİLTANGLE GOAL: %f", tiltangle_goal);
    }
    // ROS_INFO("state_1: %d", state_1);

    if (state_1 == 1 && state_2 == 0)
    {

        if (vel_state == 0)
        {
            start_time = std::chrono::high_resolution_clock::now();
            started_tilt = tiltangle;
            vel_state++;
        }

        pointcloud_output = 1; // pointcloud output is activated
        vel_tilt = -k * fabs(tiltangle_goal - tiltangle) / tiltangle_goal;
        if (fabs(vel_tilt) > vel_ref[0])
        {
            vel_tilt = vel_ref[0] * vel_tilt / fabs(vel_tilt);
        }
        // vel_pan = vel_ref[1];

        // velocity calculation

        // print angle and panangle and org
        // ROS_INFO("state_1: %d", state_1);
        // ROS_INFO("angle: %f", tiltangle);
        // ROS_INFO("panangle: %f", panangle);

        // // print vel_tilt and vel_pan
        // //ROS_INFO("vel_pan: %f", vel_pan);
        ROS_INFO("vel_tilt: %f", vel_tilt);

        if (tiltangle <= tiltangle_goal)
        {
            end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed_time = end_time - start_time;
            double angle_diff = tiltangle - 0.3059;
            measured_veltilt = angle_diff / elapsed_time.count();
            ROS_INFO("Measured tilt velocity: %f", measured_veltilt);
            ROS_INFO("Elapsed time: %f", elapsed_time.count());
            ROS_INFO("elapsed time: %f", elapsed_time);

            tiltangle_goal = -tiltangle_goal;
            state_1++;
            pointcloud_output = 0;
        }
        ROS_INFO("Second tilt movement is working FOR POINTCLOUD");
        ROS_INFO("tiltangle: %f", tiltangle);
    }

    if (state_1 == 2 && state_2 == 0)
    {

        vel_tilt = -k * fabs(tiltangle_goal - tiltangle) / tiltangle_goal;
        if (fabs(vel_tilt) > vel_ref[0])
        {
            vel_tilt = vel_ref[0] * vel_tilt / fabs(vel_tilt);
        }
        ROS_INFO("vel tilt: %f", vel_tilt);
        // vel_tilt = vel_ref[0]; //vel_ref[0];
        // vel_pan = -vel_ref[1];

        // print angle and panangle and org

        // ROS_INFO("state_2: %d", state_2);

        // ROS_INFO("panangle: %f", panangle);
        //  print vel_tilt and vel_pan
        // ROS_INFO("vel_tilt: %f", vel_tilt);
        // ROS_INFO("vel_pan: %f", vel_pan);
        // ROS_INFO("First tilt and pan movement is working");

        if (tiltangle >= 0.0)
        {

            state_1++;
        }

        ROS_INFO("angle: %f", tiltangle);
        ROS_INFO("TİLTANGLE GOAL: %f", tiltangle_goal);
    }

    if (state_1 == 3 && state_2 == 0)
    {
        vel_tilt = 0.0;
        ROS_INFO("Tilt movement stopped");
        save = 1;
        state_1++;
        if (fabs(tiltangle) <= 0.01)
        {
            vertical_scan = 1;
            state_1++;
        }
        // TILT MOVEMENT IS NOW STOPPED FOR ONE VERTICAL SCAN
    }

  

    if (pointcloud_output == 1)
    { // resolution
        if (ctn < scan_number)
        {

            scanSize = (int)laser_scan.size();

            // print the scanSize
            // ROS_INFO("scanSize: %d", scanSize);

            cloud_out.height = 1;
            cloud_out.width = scanSize * scan_number;
            cloud_out.fields.resize(3);
            cloud_out.fields[0].name = "x";
            cloud_out.fields[0].offset = 0;
            cloud_out.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            cloud_out.fields[0].count = 1;
            cloud_out.fields[1].name = "y";
            cloud_out.fields[1].offset = 4; // 4
            cloud_out.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            cloud_out.fields[1].count = 1;
            cloud_out.fields[2].name = "z";
            cloud_out.fields[2].offset = 8; // 8
            cloud_out.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            cloud_out.fields[2].count = 1;

            int offset = 12;

            cloud_out.point_step = offset;
            cloud_out.row_step = cloud_out.point_step * cloud_out.width;
            cloud_out.data.resize(cloud_out.row_step * cloud_out.height);
            cloud_out.is_dense = false;

            unsigned int count = ctn * scanSize;

            for (int i = 0; i < scanSize; i++)
            {

                // laser to end effector frame coordinate
                start_angle = 0.0;
                yy = laser_scan[i] * sin(laser_increment * (i) + start_angle);
                xx = laser_scan[i] * cos(laser_increment * (i) + start_angle);
                zz = 0;

                // kinematic map frame conversion

                x[i] = xx * (cos(panangle) * cos(tiltangle) - sin(panangle) * sin(tiltangle)) + zz * (cos(panangle) * sin(tiltangle) + cos(tiltangle) * sin(panangle)) + 0.037 * (cos(panangle) * sin(tiltangle) + cos(tiltangle) * sin(panangle)) + 0.105 * (cos(panangle) * cos(tiltangle) - sin(panangle) * sin(tiltangle)) - x1 * cos(panangle) * sin(tiltangle);
                y[i] = xx * (sin(panangle) * cos(tiltangle) + cos(panangle) * sin(tiltangle)) + zz * (sin(panangle) * sin(tiltangle) - cos(panangle) * cos(tiltangle)) + 0.037 * (sin(panangle) * sin(tiltangle) - cos(panangle) * cos(tiltangle)) + 0.105 * (sin(panangle) * cos(tiltangle) + cos(panangle) * sin(tiltangle)) - x1 * sin(panangle) * sin(tiltangle);
                ;
                z[i] = yy + 0.121;
                float *pstep = (float *)&cloud_out.data[count * cloud_out.point_step];

                pstep[0] = x[i];
                pstep[1] = y[i];
                pstep[2] = z[i];
                ++count;
            }

            ctn++;
        }

        else
        {
            cloud_out.data.clear();
            ctn = 0;
        }

        lastangle = tiltangle;
        lastpan = panangle;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_real");
    ros::NodeHandle n;

    // subscribing topics
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, encoCallback); // arduino encoder
    ros::Subscriber laserSub = n.subscribe("/scan", 1000, laserCallback);   // hokuyo data

    // publishers
    ros::Publisher pclPub = n.advertise<sensor_msgs::PointCloud2>("output", 10); // 3d data
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);  // cmd_vel commands

    geometry_msgs::Twist vel_msg;
    ros::Rate loop(50);

    while (ros::ok())
    {

        vel_msg.linear.x = vel_tilt;
        vel_msg.linear.y = vel_pan;
        vel_pub.publish(vel_msg);

        if (save == 1) // save == 1 && state_1 == 6 && state_2 == 6
        {
            // save the data to pcd ASCII
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(cloud_out, *pcl_cloud);
            pcl::io::savePCDFileASCII("Pointcloudkali6.pcd", *pcl_cloud); // we need indexing here like pointcloud_xxx.pcd
            ROS_INFO("PointCloud2 saved to pointcloud.pcd");
            state_1++;
            state_2++;
            // print the state value
            ROS_INFO("state: %d", state_1);
            save = 0;
            // print the save value
            ROS_INFO("save: %d", save);
        }

        cloud_out.header.frame_id = "map";
        cloud_out.header.stamp = last_laser.header.stamp;
        cloud_out.header.seq = cloud_out.header.seq + 1;

        pclPub.publish(cloud_out);
        ros::spinOnce();
        // ros::spin();
        loop.sleep();
    }

    return 0;
}