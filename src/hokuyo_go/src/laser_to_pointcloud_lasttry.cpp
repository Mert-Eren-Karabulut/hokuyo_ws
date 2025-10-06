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

using namespace std;

ros::Subscriber laserSub;

std::vector<float> laser_scan;
std::vector<std::vector<float>> output_matrix;

double tiltangle = 0.0;
double tilt = 0.0;
double panangle = 0.0;
double lastangle = 0.0;
double lastangle2 = 0.0;
double lastpan=0.0;
double laser_increment = 0.0;
double tilt_min = 0.0;
double tilt_max = 0.2;
double initial_height = 0.0; // Set the initial height to the starting position
double vertical_distance = 0.0;
double angle_goal_tilt = 0.87; //the desired tilt angle 10 degrees 0.184532925
double angle_goal_pan = 0.0; //the desired pan angle

float x[1024];
float y[1024];
float z[1024];
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
int scan_num=15; //1500
int scan_count=0;


//declaring the laserscan and pointcloud objects
sensor_msgs::LaserScan last_laser;
sensor_msgs::PointCloud2 cloud_out;

int save = 0; // Set this to 1 to save the pointcloud data to pcd ASCII file

// Laser data callback
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //assigning laser data
    laser_increment = msg->angle_increment;
    laser_scan = msg->ranges;
    last_laser = *msg;
    
    
    // if (scan_count == 100)
    // {
    //     ROS_INFO("Laser data received 100 times now unsubscribing...");
    //     laserSub.shutdown();
    // }
}



// Joint states callback
void encoCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    //getting joint state data

    tiltangle = msg->position[1];
    panangle= msg->position[0];

    
    
    //ROS_INFO("Angle: %.6f", angle);  // Print angle with more decimal places for accuracy
    //print lastpan
    // ROS_INFO("PanAngle: %.6f", panangle);
    // ROS_INFO("Last PanAngle: %.6f", lastpan);
    //print lastangle
    //ROS_INFO("Last Angle: %.6f", lastangle);
    //print lastangke2
    //ROS_INFO("Last Angle2: %.6f", lastangle2);
    //if i durgun halde sensör çıktısı almak için değiştirdik normal hali : if( fabs(angle-lastangle)>=(M_PI)/90 || fabs(panangle-lastpan)>=(M_PI)/720 ){
    
    
   // for tilt movement, change the if statement to this:  
   //angle-angle_goal_tilt) < 0.005 && fabs(panangle-lastpan)>=0*(M_PI)/720 


   if( fabs(panangle-lastpan)>=0*(M_PI)/720 && scan_count < 10){ 
        ROS_INFO("Processing point cloud data...");
        //print panangle
        //ROS_INFO("PanAngle: %.6f", panangle);
        
        //print angle
        //ROS_INFO("Angle: %.6f", angle);
        save = 1;
        if(ctn<scan_num){

        

            scanSize= (int)laser_scan.size();
            
            //assigning point cloud attributes
            cloud_out.height = 1;
            cloud_out.width  = scanSize*scan_num;  //scanSize = 640
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
            cloud_out.data.resize (cloud_out.row_step   * cloud_out.height); //cloud_out.data is the pointcloud data
            cloud_out.is_dense = false;

            unsigned int count = ctn*scanSize;

// 
            for(int i=0; i<scanSize; i++){

           
                start_angle=-2.094; //-119.977362
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
            
            if (ctn == 0)
            {
                initial_height = z[0];                   
            }
            else if (ctn == scan_num - 1)
            {
                vertical_distance = z[scanSize - 1] - initial_height;
                //ROS_INFO("Vertical Distance: %.6f", vertical_distance);
            }

            
            ctn++;
            angle_goal_tilt = 0;

        }
        
        
        //reseting data at certain scan number
        else{
            //ctn = 0;
            return;
            //cloud_out.data.clear();  //clearing the pointcloud data
            
        }
        
        ctn++;
        lastangle = tiltangle;
    	lastpan = panangle;
    
        // std::vector<float> row(cloud_out.data.begin(), cloud_out.data.end());
        // output_matrix.push_back(row);
        // // Now, output_matrix[i] will give you the i-th scan data
        // ROS_INFO("Size of output_matrix: %ld", output_matrix.size());
        scan_count++;
        //print scan_count
        ROS_INFO("Scan Count: %d", scan_count);
    }

    else{
        
        ROS_INFO("Point cloud processing is finished");
    }
    

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_to_pointcloud");
  ros::NodeHandle n;

 
  //subscribing topics
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, encoCallback);
  laserSub = n.subscribe("/hokuyo/scan", 1000, laserCallback); // /hokuyo/scan
  
  //publishing resulting point cloud
  ros::Publisher pclPub = n.advertise<sensor_msgs::PointCloud2> ("output", 10);
  
  
  //publishing pointcloud
  ros::Rate loop_rate(10);
  while(ros::ok()){ 

      cloud_out.header.frame_id = "taban";
      cloud_out.header.stamp = last_laser.header.stamp;
      cloud_out.header.seq = cloud_out.header.seq+1;

      pclPub.publish(cloud_out);
      ros::spinOnce();
      loop_rate.sleep();

  }
  //print the size of output_matrix
  //ROS_INFO("Size of output_matrix: %ld", output_matrix.size());



  return 0;
}
