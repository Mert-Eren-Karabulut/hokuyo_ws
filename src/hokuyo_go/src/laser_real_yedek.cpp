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
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

using namespace std;

std::vector<float> laser_scan;

double tiltangle = 0.0;
double tilt = 0.0;
double panangle = 0.0;
double lastangle = 0.0;
double lastangle2 = 0.0;
double lastpan=0;
double laser_increment = 0.0;
double vel_pan=0;
double vel_tilt=0;

float x[1024];
float y[1024];
float z[1024];
int scan_number=5000;
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
float l_2=0.075;
float xx;
float yy;
float zz;
float e_x,e_y,e_z;
float start_angle;
float x1=0.106;
float vel_ref[]={0.2,0.8};
int org=0;
float destinations[6][2]={0.55,0,0.55,2,0,-2,-0.55,0,-0.55,2,-0.55,-2};

//declaring the lasercan and pointcloud objects
sensor_msgs::LaserScan last_laser;
sensor_msgs::PointCloud2 cloud_out;
int save=0;


// Getting laser data

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

    laser_increment = msg->angle_increment;
    laser_scan = msg->ranges;

    last_laser = *msg;

}



// Encoder Callback and cmd_vel calculations
void encoCallback(const sensor_msgs::JointState::ConstPtr& msg)
{


    //getting joint states
    tiltangle = msg->position[0];
    panangle= msg->position[1];
    org = 6;
    //for loop for 6 different positions
    for(int i=0;i<6;i++){
        if(org==i && abs(tiltangle-destinations[i][0])<0.1 && abs(panangle-destinations[i][1])<0.1){
            org++;
            // cout<<"org"<<org<<endl;
            // cout<<"angle"<<angle<<endl;
            // cout<<"panangle"<<panangle<<endl;
        }
        if(org==i && abs(tiltangle-destinations[i][0])<0.1 && abs(panangle-destinations[i][1])>0.1){
            vel_tilt=0;
            vel_pan=vel_ref[1];
            // cout<<"org"<<org<<endl;
            // cout<<"angle"<<angle<<endl;
            // cout<<"panangle"<<panangle<<endl;
        }
        if(org==i && abs(tiltangle-destinations[i][0])>0.1 && abs(panangle-destinations[i][1])<0.1){
            vel_tilt=vel_ref[0];
            vel_pan=0;
            // cout<<"org"<<org<<endl;
            // cout<<"angle"<<angle<<endl;
            // cout<<"panangle"<<panangle<<endl;
        }
        if(org==i && abs(tiltangle-destinations[i][0])>0.1 && abs(panangle-destinations[i][1])>0.1){
            vel_tilt=vel_ref[0];
            vel_pan=vel_ref[1];
            // cout<<"org"<<org<<endl;
            // cout<<"angle"<<angle<<endl;
            // cout<<"panangle"<<panangle<<endl;
        }
        if(org==i && abs(tiltangle-destinations[i][0])<0.1 && abs(panangle-destinations[i][1])<0.1){
            vel_tilt=0;
            vel_pan=0;
            save=1;
            // cout<<"org"<<org<<endl;
            // cout<<"angle"<<angle<<endl;
            // cout<<"panangle"<<panangle<<endl;
        }
    }
    //one choice of movement (3-tilt)
    //this movement code needs update, you need to use APF for position control. 
    // if(org==0 && abs(angle)<0.1){
    // vel_tilt=vel_ref[0];
    // vel_pan=0;
    // }
    // if(org==0 && angle>0.55 ){
    // vel_tilt=-vel_ref[0];
    // vel_pan=0;
    // org++;
    // }
    // if(angle<-0.55 && org==1){
    // vel_tilt=0;
    // vel_pan=vel_ref[1];
    // org++;
    // }
    
    // if(panangle>2 && org==2){
    // vel_pan=0;
    // vel_tilt=vel_ref[0];
    // org++;
    // }
    // if(org==3 && angle>0.55){
    // vel_tilt=0;
    // vel_pan=-vel_ref[1];
    // org++;
    // }
    // if(org==4 && panangle <-2){
    // vel_pan=0;
    // vel_tilt=-vel_ref[0];
    // org++;
    // }
    // if(org==5&& angle<-0.55){
    // vel_tilt=0;
    // vel_pan=vel_ref[1];
    // org++;
    // }
    // if(org==6&& panangle>0){
    // vel_pan=0;
    // vel_tilt=0;
    // save=1;
    // //org=0;
    // }
    

   //Deciding on resolution, and assigning 3d data
   if( fabs(tiltangle-lastangle)>=(M_PI)/1000 ||org==6){ //resolution 
        if(ctn<scan_number){
            
            if(save==1){
            //save the data to pcd ASCII
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    	    pcl::fromROSMsg(cloud_out, *pcl_cloud);
    	    pcl::io::savePCDFileASCII("pointcloud.pcd", *pcl_cloud); //we need indexing here like pointcloud_xxx.pcd
    	    ROS_INFO("PointCloud2 saved to pointcloud.pcd");
    	    org=7;
    	    save=0;
            }

            scanSize= (int)laser_scan.size();

            
            cloud_out.height = 1;
            cloud_out.width  = scanSize*scan_number;    
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


            for(int i=0; i<scanSize; i++){
            
            	//laser to end effector frame coordinate 
                start_angle=-0.785;
                yy= laser_scan[i]*sin(laser_increment*i+start_angle);
                xx= laser_scan[i]*cos(laser_increment*i+start_angle);
                zz= 0;
                
                //kinematic map frame conversion
                x[i]=-xx*sin(panangle)-
                                    yy*cos(panangle)*cos(tiltangle)-zz*cos(panangle)*sin(tiltangle)+
                                    0.005*cos(panangle)-0.037*sin(panangle)-0.008*cos(panangle)*cos(tiltangle)-x1*cos(panangle)*sin(tiltangle);
                                    
                y[i]=xx*cos(panangle)-
                                    yy*cos(tiltangle)*sin(panangle)-zz*sin(panangle)*sin(tiltangle)+
                                    0.037*cos(panangle)+0.005*sin(panangle)-0.008*cos(tiltangle)*sin(panangle)-x1*sin(panangle)*sin(tiltangle);
                z[i]=-yy*sin(tiltangle)+zz*cos(tiltangle)+x1*cos(tiltangle)-sin(tiltangle)*0.008+0.121;
                
                
                
                
     
       

                float *pstep = (float*)&cloud_out.data[count * cloud_out.point_step];

                
                
                pstep[0] = x[i];
                pstep[1] = y[i];
                pstep[2] = z[i];
                ++count;

            }
            
            ctn++;
        }
        
        else{
        cloud_out.data.clear();
        ctn=0;
        }

        ctn++;
        lastangle=tiltangle;
        lastpan=panangle;
    }
    

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_real");
  ros::NodeHandle n;

 
  //subscribing topics
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, encoCallback); //arduino encoder
  ros::Subscriber laserSub = n.subscribe("/scan", 1000, laserCallback);   //hokuyo data
  
  //publishers
  ros::Publisher pclPub = n.advertise<sensor_msgs::PointCloud2> ("output", 10); //3d data
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);   //cmd_vel commands


  geometry_msgs::Twist vel_msg;
  ros::Rate loop(10);
  while(ros::ok()){
      
      vel_msg.linear.x=vel_tilt;
      vel_msg.linear.y=vel_pan;
      vel_pub.publish(vel_msg);

      cloud_out.header.frame_id = "map";
      cloud_out.header.stamp = last_laser.header.stamp;
      cloud_out.header.seq = cloud_out.header.seq+1;

      pclPub.publish(cloud_out);
      cout << "publishing" << endl;
      
      ros::spinOnce();
      loop.sleep();

  }



  return 0;
}
