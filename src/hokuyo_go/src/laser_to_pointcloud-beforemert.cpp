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

#include "utility.h"

using namespace std;

Conncomp1d conncomp;
StdDeviation stddev;

std::vector<float> laser_scan;

double tiltangle = 0.0;
double tilt = 0.0;
double panangle = 0.0;
double lastangle = 0.0;
double lastangle2 = 0.0;
double lastpan = 0;
double laser_increment = 0.0;
double max_laser = 0.0;
double min_laser = 0.0;
double tilt_min = 0.0;
double tilt_max = 0.2;

float x[1024];
float y[1024];
float z[1024];
int ctn = 0;
int t = 0;
int scanSize;
int a = 1;
int b = 0;
int d1 = 0.05;
int d2 = 0.1;
float d_1 = 0.114;
float d_2 = 0.02845;
float l_1 = 0.045;
float l_2 = 0.040; // 0.075
float xx;
float yy;
float zz;
float e_x, e_y, e_z;
float start_angle;
float center_comp_angle = 0.0;
int scan_num = 500;
double max_center_value = 0.0;
int max_center_index = -1;
int max_component_width = 0.0;
int start_index = -1;
int end_index = -1;
int component_width;
double component_centroid;
int centroid_index;

std::vector<float> center_values;

// declaring the lasercan and pointcloud objects
sensor_msgs::LaserScan last_laser;
sensor_msgs::PointCloud2 cloud_out;

// Laser data callback
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // assigning laser data
    laser_increment = msg->angle_increment;
    laser_scan = msg->ranges;
    max_laser = msg->angle_max;
    min_laser = msg->angle_min;

    // print the laser inrement
    // ROS_INFO("laser increment: %.6f", laser_increment);

    // ROS_INFO("max laser angle: %.6f", max_laser);
    // ROS_INFO("min laser angle: %.6f", min_laser);

    int laser_count = laser_scan.size();

    std::vector<float> laser_values(laser_scan.begin(), laser_scan.end());
    int binaryImage[laser_count];

    stddev.SetValues(laser_values, laser_count);               // Set laser data for standard deviation calculation
    double standard_deviation = stddev.GetStandardDeviation(); // Calculate standard deviation
    double threshold = standard_deviation / 2;                 // Set threshold as half the standard deviation

    // set binay image first using only detected image ranges
    for (int i = 0; i < laser_count; ++i)
    {
        if (laser_scan[i] < 6.0)
        {
            binaryImage[i] = 1; // Object detected within the threshold
        }
        else
        {
            binaryImage[i] = 0; // No object detected or beyond threshold
        }
    }

    // change the binary image values using threshold between two consecutive laser data
    for (int i = 1; i < laser_count; ++i)
    {
        if (fabs(laser_scan[i] - laser_scan[i - 1]) > threshold)
        {
            // ROS_INFO("Difference between two consecutive laser data is beyond threshold");
            binaryImage[i] = 0; // Set as 0 if the difference between two consecutive laser data is beyond threshold
        }
    }

    conncomp.SetValues1(binaryImage, laser_count);

    // determine the number of connected components
    int num_components = conncomp.findcompNumber();
    // process connected component width and centroid
    conncomp.findcompwidth();
    conncomp.findcentroid();

    // access component information
    for (int i = 0; i < num_components; ++i)
    {
        component_width = conncomp.compwidth[i];
        component_centroid = conncomp.centroid[i];

        // process or store component width and centroid as needed
        // ROS_INFO("Component %d: Width = %.6f, Centroid = %.6f", i+1, component_width, component_centroid);

        // get the nearest integer to component_centroid
        centroid_index = (int)component_centroid;
        // ROS_INFO("laser value at the center of the component: %.6f", laser_scan[centroid_index]);
        // ROS_INFO("centroid index: %d", centroid_index);
        // form an array for center values of the cmponents
        center_values.push_back(laser_scan[centroid_index]);

        // compute the biggest value in the center values
        max_center_value = *std::max_element(center_values.begin(), center_values.end());

        // Determine the index of the maximum center value
        if (laser_scan[centroid_index] == max_center_value)
        {
            max_center_index = centroid_index;
            max_component_width = component_width;
        }

        if (centroid_index == max_center_index)
        {
            // first component
            start_index = max_center_index - component_width / 2;
            end_index = max_center_index + component_width / 2;
        }
    }
    center_comp_angle = max_center_index * laser_increment;
    // ROS_INFO("Number of connected components: %d", num_components); // Print the number of connected components
    // ROS_INFO("Farthest component's center laser value: %.6f meters", max_center_value);
    // ROS_INFO("Farthest component's center laser angle value: %.6f radians", center_comp_angle);
    // ROS_INFO("Start index of the farthest component: %d", start_index);
    // ROS_INFO("End index of the farthest component: %d", end_index);
    // ROS_INFO("Farthest component's starting angle: %.6f radians", start_index * laser_increment);
    // ROS_INFO("Farthest component's ending angle: %.6f radians", end_index * laser_increment);

    // use center_comp_angle for determining the speed of the robot

    last_laser = *msg;
}

// Joint states callback
void encoCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    // getting joint state data

    tiltangle = msg->position[1];
    panangle = msg->position[0];

    // ROS_INFO("Angle: %.6f", angle);  // Print angle with more decimal places for accuracy
    // print lastangle
    // ROS_INFO("Last Angle: %.6f", lastangle);
    // print lastangke2
    // ROS_INFO("Last Angle2: %.6f", lastangle2);
    // if i durgun halde sensör çıktısı almak için değiştirdik normal hali : if( fabs(angle-lastangle)>=(M_PI)/90 || fabs(panangle-lastpan)>=(M_PI)/720 ){

    if (fabs(tiltangle - lastangle) >= (M_PI)/90  && fabs(panangle - lastpan) >= 0 * (M_PI) / 720)
    {
        // ROS_INFO("Processing point cloud data...");
        if (ctn < scan_num)
        {

            scanSize = (int)laser_scan.size();

            // assigning point cloud attributes
            cloud_out.height = 1;
            cloud_out.width = scanSize * scan_num;
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

            //
            for (int i = 0; i < scanSize; i++)
            {

                start_angle = -2.094;
                yy = laser_scan[i] * sin(laser_increment * i + start_angle);
                xx = laser_scan[i] * cos(laser_increment * i + start_angle);
                zz = 0;
                // simülasyon için değiştirilecek
                x[i] = xx * cos(panangle) -
                       yy * sin(panangle) * cos(tiltangle) + zz * sin(panangle) * sin(tiltangle) +
                       l_1 * sin(panangle) * sin(tiltangle) + l_2 * sin(panangle);

                y[i] = xx * sin(panangle) +
                       yy * cos(panangle) * cos(tiltangle) - zz * cos(panangle) * sin(tiltangle) -
                       l_1 * cos(panangle) * sin(tiltangle) + l_2 * cos(panangle) - d_2;
                z[i] = yy * sin(tiltangle) + zz * cos(tiltangle) + l_1 * cos(tiltangle) + d_1;
                // değiştirme

                // ROS_INFO("xx: %.6f", xx);
                // ROS_INFO("yy: %.6f", yy);
                // ROS_INFO("zz: %.6f", zz);
                // ROS_INFO("x: %.6f", x[i]);
                // ROS_INFO("y: %.6f", y[i]);
                // ROS_INFO("z: %.6f", z[i]);
                // ROS_INFO("i: %d", i);

                float *pstep = (float *)&cloud_out.data[count * cloud_out.point_step];

                pstep[0] = x[i]; // pointclouda ekleme
                pstep[1] = y[i];
                pstep[2] = z[i];
                ++count;
            }

            ctn++;
        }

        // reseting data at certain scan number
        else
        {
            ROS_INFO("Point cloud data processed. CTN limit reached.");
        }

        // ctn++;
        lastangle = tiltangle;
        lastpan = panangle;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_to_pointcloud");
    ros::NodeHandle n;

    // subscribing topics
    ros::Subscriber sub = n.subscribe("/joint_states", 1000, encoCallback);
    ros::Subscriber laserSub = n.subscribe("/hokuyo/scan", 1000, laserCallback); // hokuyo/scan

    // publishing resulting point cloud
    ros::Publisher pclPub = n.advertise<sensor_msgs::PointCloud2>("output", 10);

    // publishing pointcloud
    ros::Rate loop(10);
    while (ros::ok())
    {

        cloud_out.header.frame_id = "taban";
        cloud_out.header.stamp = last_laser.header.stamp;
        cloud_out.header.seq = cloud_out.header.seq + 1;

        pclPub.publish(cloud_out);
        ros::spinOnce();
        loop.sleep();
    }

    return 0;
}
