//#include <string>
#include "gazebo_msgs/SpawnModel.h"
#include <gazebo_msgs/SpawnModelRequest.h>
#include <gazebo_msgs/SpawnModelResponse.h>
//#include "gazebo/GetWorldProperties.h"
#include "gazebo_msgs/DeleteModel.h"
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <boost/filesystem.hpp>

//#include "geometry_msgs/Pose.h"
//#include "std_srvs/Empty.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
//#include <tinyxml.h>

// Including ros, just to be able to call ros::init(), to remove unwanted
// args from command-line.
#include <ros/ros.h>
#include <chrono>
#include <thread>
#include <odom_publisher.h>
#include <vector>

#define LOOP_RATE 10
namespace fsys = boost::filesystem;
using namespace std;

double ROB_INIT_POS_X = -3.0;
double ROB_INIT_POS_Y = -3.0;
double ROB_INIT_POS_Z = 0;
double CIRCULAR_RAD = 3.0;
int NSTEPS = 100;

std::string yaml_file_path="/home/mali/turtlebot_ws/src/gazebo_sim/config/rosparam.yaml";
std::string urdf_file_path="/home/mali/turtlebot_ws/src/gazebo_sim/urdf/robot2.urdf.xacro";

typedef struct {
    double x;
    double y;
    double rho;
    double h;

}  obstacle;

vector<obstacle>  allObstacles;


namespace gazebo_test_tools {

/**
 * Spawns a cube/primitive of given size, position and orientation into
 * Gazebo.
 *
 * \author Jennifer Buehler
 src = "https://github.com/JenniferBuehler/gazebo-pkgs/blob/noetic/gazebo_test_tools/src/cube_spawner_node.cpp"
 */
class GazeboCubeSpawner {
public:
    GazeboCubeSpawner(ros::NodeHandle &n);

    void spawnCube(const std::string& name, const std::string& frame_id,
            float x, float y, float z, float qx, float qy, float qz, float qw,
            float w=0.05, float h=0.05, float d=0.05, float mass=0.05);

    /**
     * \param isCube if true, spawn a cube. If false, spawn cylinder,
     *      where \e w is the radius and \e h is the height (\e d will be ignored).
     */
    void spawnPrimitive(const std::string& name, const bool isCube,
            const std::string& frame_id,
            float x, float y, float z, float qx, float qy, float qz, float qw,
            float w=0.05, float h=0.05, float d=0.05, float mass=0.05);

private:

    ros::NodeHandle nh;
    ros::ServiceClient spawn_object;

};

}  // namespace


using gazebo_test_tools::GazeboCubeSpawner;
#define SPAWN_OBJECT_TOPIC "gazebo/spawn_sdf_model"


//GazeboCubeSpawner::GazeboCubeSpawner(NodeHandle &n : nh(n){
//    spawn_object = n.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);
//}

GazeboCubeSpawner::GazeboCubeSpawner(ros::NodeHandle &n){
    spawn_object = n.serviceClient<gazebo_msgs::SpawnModel>(SPAWN_OBJECT_TOPIC);
}


void GazeboCubeSpawner::spawnCube(const std::string& name, const std::string& frame_id,
    float x, float y, float z, float qx, float qy, float qz, float qw,
    float width, float height, float depth, float mass)
{
    spawnPrimitive(name, true, frame_id, x, y, z, qx, qy, qz, qw, width, height, depth, mass);
}

void GazeboCubeSpawner::spawnPrimitive(const std::string& name, const bool doCube,
    const std::string& frame_id,
    float x, float y, float z, float qx, float qy, float qz, float qw,
    float widthOrRadius, float height, float depth, float _mass)
{

    geometry_msgs::Pose pose;
    pose.position.x=x;
    pose.position.y=y;
    pose.position.z=z;
    pose.orientation.x=qx;
    pose.orientation.y=qy;
    pose.orientation.z=qz;
    pose.orientation.w=qw;

    gazebo_msgs::SpawnModel spawn;
    spawn.request.model_name=name;

    // just so the variable names are shorter..
    float w=widthOrRadius;
    float h=height;
    float d=depth;

    std::stringstream _s;
    if (doCube)
    {
          _s<<"<box>\
            <size>"<<w<<" "<<h<<" "<<d<<"</size>\
          </box>";
    }else{

          _s<<"<cylinder>\
                <length>"<<h<<"</length>\
                <radius>"<<w<<"</radius>\
            </cylinder>";
    }
    std::string geometryString = _s.str();


    float mass=_mass;
    float mass12=mass/12.0;

    double mu1=500; //500 for PR2 finger tip. In first experiment had it on 1000000
    double mu2=mu1;
    double kp=10000000; //10000000 for PR2 finger tip
    double kd=1; //100 for rubber? 1 fir OR2 finger tip

    bool do_surface=false;
    bool do_inertia=true;

    std::stringstream s;\
    s<<"<?xml version='1.0'?>\
    <sdf version='1.4'>\
    <model name='"<<name<<"'>\
        <static>false</static>\
        <link name='link'>";

    // inertia according to https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    if (do_inertia)
    {
        double xx, yy, zz;
        if (doCube)
        {
            xx=mass12*(h*h+d*d);
            yy=mass12*(w*w+d*d);
            zz=mass12*(w*w+h*h);
        }
        else
        {
            xx=mass12*(3*w*w + h*h);
            yy=mass12*(3*w*w + h*h);
            zz=0.5*mass*w*w;
        }
        s<<"<inertial>\
        <mass>"<<mass<<"</mass>\
        <inertia>\
          <ixx>"<<xx<<"</ixx>\
          <ixy>0.0</ixy>\
          <ixz>0.0</ixz>\
          <iyy>"<<yy<<"</iyy>\
          <iyz>0.0</iyz>\
          <izz>"<<zz<<"</izz>\
        </inertia>\
          </inertial>";
    }
    s<<"<collision name='collision'>\
        <geometry>"<<geometryString;
    s<<"</geometry>";
    if (do_surface)
        s<<"<surface>\
            <friction>\
              <ode>\
            <mu>"<<mu1<<"</mu>\
            <mu2>"<<mu2<<"</mu2>\
            <fdir1>0.000000 0.000000 0.000000</fdir1>\
            <slip1>0.000000</slip1>\
            <slip2>0.000000</slip2>\
              </ode>\
            </friction>\
            <bounce>\
              <restitution_coefficient>0.000000</restitution_coefficient>\
              <threshold>100000.000000</threshold>\
            </bounce>\
            <contact>\
              <ode>\
            <soft_cfm>0.000000</soft_cfm>\
            <soft_erp>0.200000</soft_erp>\
            <kp>"<<kp<<"</kp>\
            <kd>"<<kd<<"</kd>\
            <max_vel>100.000000</max_vel>\
            <min_depth>0.001000</min_depth>\
              </ode>\
            </contact>\
        </surface>";
      s<<"</collision>\
          <visual name='visual'>";
      s<<"<geometry>"<<geometryString;
      s<<"</geometry>\
        <material>\
            <script>\
                <uri>file://media/materials/scripts/gazebo.material</uri> \
                <name>Gazebo/Red</name>\
            </script>\
        </material>\
          </visual>\
        </link>\
      </model>\
    </sdf>";

    spawn.request.model_xml=s.str();
    spawn.request.robot_namespace="cube_spawner";
    spawn.request.initial_pose=pose;
    spawn.request.reference_frame=frame_id;

    //ROS_INFO("Resulting model: \n %s",s.str().c_str());

    //ROS_INFO("Waiting for service");
    spawn_object.waitForExistence();
    //ROS_INFO("Calling service");

    //std::cout<<spawn.request<<std::endl;

    if (!spawn_object.call(spawn)) {
        ROS_ERROR("Failed to call service %s",SPAWN_OBJECT_TOPIC);
    }
    ROS_INFO("Result: %s, code %u",spawn.response.status_message.c_str(), spawn.response.success);
}


void readObstacles(std::string& filename){

  std::ifstream myFile(filename);
  if(!myFile.is_open()){
    cout << "ERROR! File could not be opened." << endl;
  }else{
    cout << "File opened successfully." << endl;
  }

  std::string(mystr);
  int M;
  myFile >> M;
  cout << "# of cylindrical obstacles is " << M << "." << endl;

  double temp = 0;
  std::getline(myFile, mystr);
  while(std::getline(myFile, mystr)){
    obstacle newobs;
    std::stringstream ss(mystr);
    ss >>  newobs.x;
    ss >> newobs.y;
    ss >> newobs.rho;
    ss >> newobs.h;
    allObstacles.push_back(newobs);
  }
  myFile.close();

  std::cout << "*** " << allObstacles.size()<< std::endl;

  for(int i = 0; i < M; i++){
    cout << "Obstacle: " << i+1 << " x:" << allObstacles.at(i).x << " y: " << allObstacles.at(i).y <<
            " rad: " << allObstacles.at(i).rho << " h: " << allObstacles.at(i).h << std::endl;
  }

  cout << "All obstacles read " << std::endl;
}





/*
 *
 * String2Char conversion
 *
 *
 */
char* string2Char(string temp) {
  stringstream strs;
  strs << temp;
  string temp_str = strs.str();
  char* char_type = (char*) temp_str.c_str();
  return char_type;
}

// Service client for setting model poses
ros::ServiceClient set_model_state_client;


/*
 *
 * Set the model state
 *
 *
 */
// src = "https://https://gist.github.com/histvan95/261482184e36bb238d9c45a361586316"
void set_model_state(std::string model_name,
                    std::string reference_frame,
                    geometry_msgs::Pose pose,
                    geometry_msgs::Twist model_twist)
{
    // Set model service struct
    gazebo_msgs::SetModelState setmodelstate;

    // Model state msg
    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = model_name;
    modelstate.reference_frame = reference_frame;
    modelstate.pose = pose;
    modelstate.twist = model_twist;

    setmodelstate.request.model_state = modelstate;

    // Call the service
    bool success = set_model_state_client.call(setmodelstate);
    if (success)
    {
        //ROS_INFO_STREAM("Setting position of " << model_name << "  was successful.");
    }
    else
    {
        ROS_ERROR_STREAM("Setting position of " << model_name << " failed.");
    }
}

void readObstaclesYaml(ros::NodeHandle &getParamHandle){

  //load the rosparam2.yaml parameters to param server
  fsys::path cfg_file_path{yaml_file_path};
  if(!fsys::exists(cfg_file_path) ||
     !(fsys::is_regular_file(cfg_file_path) || fsys::is_symlink(cfg_file_path)))
  {
    ROS_WARN_STREAM("Could not open the param file " << cfg_file_path.string());
  } else {
    // load the YAML file using the external rosparam command
    std::string command = "rosparam load " + cfg_file_path.string();
    int result = std::system(command.c_str());
    if(result != 0){
      ROS_WARN_STREAM("Could not set config file " << cfg_file_path.string()
                      << " to the parameter server.");
    }
  }
  
  //read the obstacles
  int M;
  vector<double> posvec;
  vector<double> rovec;
  vector<double> hvec;
  vector<double> start_vec;
  getParamHandle.getParam("/robpos",start_vec);
  ROB_INIT_POS_X=start_vec.at(0);
  ROB_INIT_POS_Y=start_vec.at(1);
  getParamHandle.getParam("N", M);
  ROS_INFO("Parameters are loaded");
  getParamHandle.getParam("/obspos", posvec);
  getParamHandle.getParam("/obsro",rovec);
  getParamHandle.getParam("/obsh",hvec);
  for (int i=0; i<M; i++){
  	obstacle newobs;
  	newobs.x=posvec[i*2]; newobs.y=posvec[i*2+1];
  	newobs.rho=rovec[i];
  	newobs.h=hvec[i];
        allObstacles.push_back(newobs);
  }
  ROS_INFO("Number of cylindrical obstacles is %d",M);

  double temp = 0;

  std::cout << "Number of read obstacles " << allObstacles.size()<< std::endl;

  for(int i = 0; i < M; i++){
    cout << "Obstacle: " << i+1 << " x:" << allObstacles.at(i).x << " y: " << allObstacles.at(i).y <<
            " rad: " << allObstacles.at(i).rho << " h: " << allObstacles.at(i).h << std::endl;
  }

  cout << "All obstacles read " << std::endl;
}



/*
 *
 * Main routine
 *
 *
 */
 
geometry_msgs::Pose model_position;
geometry_msgs::Twist model_twist;


void pos_callback(const geometry_msgs::Pose& msg){
    model_position.position.x=msg.position.x;
    model_position.position.y=msg.position.y;
    model_position.position.z=0;
    model_position.orientation.x = 0.0;
    model_position.orientation.y = 0.0;
    model_position.orientation.z = 0.0;
    model_position.orientation.w = 0.0;

}

void vel_callback(const geometry_msgs::Twist& msg){
   
    model_twist.linear.x = msg.linear.x;
    model_twist.linear.y = msg.linear.y;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;

}

int main(int argc, char** argv)

{

    

    ros::init(argc, argv, "my_node2");
    ros::NodeHandle nh;
    ros::ServiceClient gazebo_spawn_clt = nh.serviceClient<gazebo_msgs::SpawnModel> ("/gazebo/spawn_urdf_model");
    gazebo_msgs::SpawnModel model;


    std::ifstream file(urdf_file_path);

    std::string line;

    while(!file.eof()) // Parse the contents of the given urdf in a string

    {
        std::getline(file,line);
        model.request.model_xml+=line;
    }

    file.close();


    //model.request.model_name="minik";
    string robot_name;
    readObstaclesYaml(nh);
    vector<double> posvecc;
    nh.getParam("/robpos_follower",posvecc);
    ROB_INIT_POS_X = posvecc.at(0);
    ROB_INIT_POS_Y = posvecc.at(1);
    ROB_INIT_POS_Z = 0;



    robot_name = "follower";
    cout << "Constructing robot "  << " -->" << robot_name <<  endl;

    model.request.model_name=(char*) robot_name.c_str();

    model.request.reference_frame="world";
    model.request.initial_pose.position.x = ROB_INIT_POS_X;
    model.request.initial_pose.position.y = ROB_INIT_POS_Y;
    gazebo_spawn_clt.call(model); //Call the service


    std::this_thread::sleep_for(std::chrono::milliseconds(3000)); //Wait 3 seconds
    
    //default values of model states
    model_position.position.x=ROB_INIT_POS_X;
    model_position.position.y=ROB_INIT_POS_Y;
    model_position.position.z=0;
    model_position.orientation.x = 0.0;
    model_position.orientation.y = 0.0;
    model_position.orientation.z = 0.0;
    model_position.orientation.w = 0.0;
   
    model_twist.linear.x = 0.001;
    model_twist.linear.y = 0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;


    int id = 0;
    //cout << "odom_publisher_node" << endl;
    //OdomPublisher* odom_publisher = new OdomPublisher(id);
    //cout << "Odometry robot " << id <<  endl;
    //odom_publisher->work();


    cout << "Create service client for setting Gazebo model state" << endl;
    // Create service client for setting Gazebo model state
    set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    gazebo_test_tools:GazeboCubeSpawner spawner(nh);
    ROS_INFO("Spawning obstacles...");

    //readObstacles(sceneFile);  // read input file
    //readObstaclesYaml(nh);
    int M = allObstacles.size();
    
    ros::Subscriber pos_sub= nh.subscribe("/cmd_pos",1000,pos_callback);
    ros::Subscriber vel_sub= nh.subscribe("/cmd_vel",1000,vel_callback);
    ros::Rate loop_rate(LOOP_RATE);
    while (ros::ok()){
    //ROS_INFO("Pos command is: (%f, %f)", model_position.position.x, model_position.position.y);
    //set_model_state("minik0", "world", model_position, model_twist);
    ros::spinOnce();
    loop_rate.sleep();
    }
   std::cout << "Hit any key to end the simulation \n";
   std::cin.get();
  //DELETE MODEL
   ros::ServiceClient gazebo_delete_clt = nh.serviceClient<gazebo_msgs::DeleteModel> ("/gazebo/delete_model");
   gazebo_msgs::DeleteModel delete_model;

   //model.request.model_name=(char*) robot_name.c_str();
   cout << "Deleting robot  -->" << robot_name <<  endl;
   delete_model.request.model_name = (char*) robot_name.c_str();
       //delete_model.request.model_name = "minik1";

   gazebo_delete_clt.call(delete_model);

   for(int i = 0; i < M; i++){
     std::string name = "obstacle" + std::to_string(i);
     cout << "Deleting obstacle  -->" << name <<  endl;
     delete_model.request.model_name = (char*) name.c_str();
     gazebo_delete_clt.call(delete_model);
     std::this_thread::sleep_for(std::chrono::milliseconds(100)); //Wait 100ms
   }



  return 0;

}
