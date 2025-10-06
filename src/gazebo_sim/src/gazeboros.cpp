 /*
  *  Gazebo - Outdoor Multi-Robot Simulator
  *  Copyright (C) 2003
  *     Nate Koenig & Andrew Howard
  *
  *  This program is free software; you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation; either version 2 of the License, or
  *  (at your option) any later version.
  *
  *  This program is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  *  GNU General Public License for more details.
  *
  *  You should have received a copy of the GNU General Public License
  *  along with this program; if not, write to the Free Software
  *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  *
  */
 
 /* Desc: External interfaces for Gazebo
  * Author: John Hsu adapted original gazebo main.cc by Nate Koenig
  * Date: 25 Apr 2010
  * SVN: $Id: main.cc 8598 2010-03-22 21:59:24Z hsujohnhsu $
  */
 
 #include <stdio.h>
 #include <stdlib.h>
 #include <signal.h>
 #include <errno.h>
 #include <iostream>
 
 // urdf utilities
 #include <tinyxml.h>
 #include <gazebo/urdf2gazebo.h>
 
 #include "gazebo/gazebo.h"
 #include "gazebo/gazebo_config.h"
 #include "gazebo/Simulator.hh"
 #include "gazebo/GazeboConfig.hh"
 #include "gazebo/GazeboError.hh"
 #include "gazebo/Global.hh"
 #include "gazebo/World.hh"
 #include "gazebo/Entity.hh"
 #include "gazebo/Model.hh"
 #include "gazebo/Body.hh"
 #include "gazebo/Geom.hh"
 #include "gazebo/Joint.hh"
 #include "gazebo/Mass.hh"
 #include "gazebo/Pose3d.hh"
 #include "gazebo/Vector3.hh"
 #include "gazebo/Quatern.hh"
 #include "gazebo/PhysicsEngine.hh"
 #include "gazebo/Mass.hh"
 
 // ros
 #include <ros/ros.h>
 #include <ros/callback_queue.h>
 #include <ros/subscribe_options.h>
 #include <ros/package.h>
 #include <rosgraph_msgs/Clock.h>
 
 // Messages
 #include <std_msgs/Bool.h>
 
 // Services
 #include "std_srvs/Empty.h"
 
 #include "gazebo_msgs/JointRequest.h"
 #include "gazebo_msgs/BodyRequest.h"
 
 #include "gazebo_msgs/SpawnModel.h"
 #include "gazebo_msgs/DeleteModel.h"
 
 #include "gazebo_msgs/ApplyBodyWrench.h"
 
 #include "gazebo_msgs/SetPhysicsProperties.h"
 #include "gazebo_msgs/GetPhysicsProperties.h"
 
 #include "gazebo_msgs/SetJointProperties.h"
 
 #include "gazebo_msgs/GetWorldProperties.h"
 
 #include "gazebo_msgs/GetModelProperties.h"
 #include "gazebo_msgs/GetModelState.h"
 #include "gazebo_msgs/SetModelState.h"
 
 #include "gazebo_msgs/GetJointProperties.h"
 #include "gazebo_msgs/ApplyJointEffort.h"
 
 #include "gazebo_msgs/GetLinkProperties.h"
 #include "gazebo_msgs/SetLinkProperties.h"
 #include "gazebo_msgs/SetLinkState.h"
 #include "gazebo_msgs/GetLinkState.h"
 
 // Topics
 #include "gazebo_msgs/ModelState.h"
 #include "gazebo_msgs/LinkState.h"
 #include "gazebo_msgs/ModelStates.h"
 #include "gazebo_msgs/LinkStates.h"
 
 #include "geometry_msgs/Vector3.h"
 #include "geometry_msgs/Wrench.h"
 #include "geometry_msgs/Pose.h"
 #include "geometry_msgs/Twist.h"
 
 // For physics dynamics reconfigure
 #include <dynamic_reconfigure/server.h>
 #include <gazebo/PhysicsConfig.h>
 #include "gazebo_msgs/SetPhysicsProperties.h"
 #include "gazebo_msgs/GetPhysicsProperties.h"
 
 // For model pose transform to set custom joint angles
 #include <ros/ros.h>
 #include <urdf/model.h>
 #include "LinearMath/btTransform.h"
 #include "LinearMath/btVector3.h"
 #include <gazebo_msgs/SetModelConfiguration.h>
 #include <boost/shared_ptr.hpp>
 #include <boost/algorithm/string.hpp>
 
 #include <tf/transform_broadcaster.h>
 
 // Command line options
 const char *worldFileName;
 const char *optLogFileName = NULL;
 unsigned int optServerId = 0;
 bool optServerForce = true;
 bool optGuiEnabled = true;
 bool optRenderEngineEnabled = true;
 double optTimeout = -1;
 unsigned int optMsgLevel = 1;
 int optTimeControl = 1;
 bool optPhysicsEnabled  = true;
 bool optPaused = false;
 bool optWorldParam = false;
 const char *worldParamName = NULL;
 std::string worldParamData;
 bool optOgreLog = false;
 
 // TODO: Implement these options
 void PrintUsage()
 {
   fprintf(stderr, "Usage: gazeboros [-hv] <worldfile>\n");
   fprintf(stderr, "  -h            : Print this message.\n");
   fprintf(stderr, "  -s <id>       : Use server id <id> (an integer); default is 0. If <0, shared memory iface disabled\n");
   fprintf(stderr, "  -f            : Force usage of the server id (use with caution)\n");
   fprintf(stderr, "  -d <-1:9>      : Verbose mode: -1 = none, 0 = critical (default), 9 = all)\n");
   fprintf(stderr, "  -t <sec>      : Timeout and quit after <sec> seconds\n");
   fprintf(stderr, "  -g            : Run without a GUI\n");
   fprintf(stderr, "  -r            : Run without a rendering engine\n");
   fprintf(stderr, "  -l <logfile>  : Log to indicated file.\n");
   fprintf(stderr, "  -n            : Do not do any time control\n");
   fprintf(stderr, "  -p            : Run without physics engine\n");
   fprintf(stderr, "  -u            : Start the simulation paused\n");
   fprintf(stderr, "  -w <param>    : World file ROS parameter name\n");
   fprintf(stderr, "  -o            : Create an Ogre.log file\n");
   fprintf(stderr, "  <worldfile>   : load the the indicated world file\n");
   ros::shutdown();
   return;
 }
 
 // Print the version/licence string
 void PrintVersion()
 {
   fprintf(stderr, "Gazebo multi-robot simulator, version %s\n\n", GAZEBO_VERSION);
   fprintf(stderr, "Part of the Player/Stage Project "
           "[http://playerstage.sourceforge.net].\n");
   fprintf(stderr, "Copyright (C) 2003 Nate Koenig, Andrew Howard, and contributors.\n");
   fprintf(stderr, "Released under the GNU General Public License.\n\n");
   return;
 }
 
 // Parse the argument list.  Options are placed in static variables.
 int ParseArgs(int argc, char **argv)
 {
   int ch;
   char *flags = (char*)("lw:hd:s:fgxt:nqperuo");
 
   // Get letter options
   while ((ch = getopt(argc, argv, flags)) != -1)
   {
     switch (ch)
     {
       case 'o':
         optOgreLog = true;
         break;
 
       case 'u':
         optPaused = true;
         break;
 
       case 'd':
         // Verbose mode
         optMsgLevel = atoi(optarg);
         break;
 
       case 'f':
         // Force server id
         optServerForce = true;
         break;
 
       case 'l':
         optLogFileName = optarg;
         break;
 
       // Get world file from parameter server
       case 'w':
         optWorldParam = true;
         worldParamName = optarg;
 
         if (!ros::param::get(std::string(worldParamName), worldParamData))
         {
                 ROS_FATAL("Unable to retrieve world parameter \"%s\" from server!", worldParamName);
                 PrintUsage();
                 return -1;
         }
 
       case 's':
         // Server id
         optServerId = atoi(optarg);
         optServerForce = false;
         break;
 
       case 't':
         // Timeout and quit after x seconds
         optTimeout = atof(optarg);
         break;
 
       case 'n':
         optTimeControl = 0;
         break;
 
       case 'g':
         optGuiEnabled = false;
         break;
 
       case 'r':
         optRenderEngineEnabled = false;
         break;
 
       case 'p':
         optPhysicsEnabled = false;
         break;
 
       case 'h':
       default:
         PrintUsage();
         return -1;
     }
   }
 
   argc -= optind;
   argv += optind;
 
   if (argc < 1 && !optWorldParam)
   {
         ROS_FATAL("No world file argument specified");
     PrintUsage();
     return -1;
   }
 
   // Get the world file name
   if (argc > 0)
           worldFileName = argv[0];
 
   return 0;
 }
 
 // sighandler to shut everything down properly
 void SignalHandler( int /*dummy*/ )
 {
   /* wait for every other nodes to shutdown before shutting down gazebo
      FIXME: should only wait for nodes associated with gazebo and plugins, is that even possible?
   ros::V_string nodes;
   ros::master::getNodes(nodes);
   while ((int)nodes.size() > 2)
   {
     nodes.clear();
     ros::master::getNodes(nodes);
     ROS_DEBUG("Trying to shutdown nicely, [%d] nodes still alive, waiting...",(int)nodes.size());
     for (ros::V_string::iterator it = nodes.begin(); it != nodes.end(); it++)
     {
       ROS_DEBUG("    node[%d]: [%s]",(int)nodes.size(),it->c_str());
       usleep(100000);
     }
 
   }
   ROS_DEBUG("Stopping gazebo");
   */
 
   //TODO: use a boost::signal
   gazebo::Simulator::Instance()->SetUserQuit();
   return;
 }
 
 
 class GazeboROSNode
 {
   public:
     GazeboROSNode() : rosnode_("~")
     {
         pub_gazebo_paused_ = this->rosnode_.advertise<std_msgs::Bool>("paused", 1, true);
 
       this->physics_reconfigure_initialized_ = false;
       this->gazebo_callback_queue_thread_ = new boost::thread( &GazeboROSNode::gazeboQueueThread,this );
 
       this->physics_reconfigure_thread_ = new boost::thread(boost::bind(&GazeboROSNode::PhysicsReconfigureNode, this));
 
       this->AdvertiseServices();
 
       // connect helper function to signal for scheduling torque/forces, etc
       gazebo::World::Instance()->ConnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::wrenchBodySchedulerSlot,this));
       gazebo::World::Instance()->ConnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::forceJointSchedulerSlot,this));
       gazebo::World::Instance()->ConnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::publishSimTime,this));
 
       // for internal gazebo xml use
       this->xmlPrefix_ = std::string("<?xml version='1.0'?> <gazebo:world xmlns:xi='http://www.w3.org/2001/XInclude' xmlns:gazebo='http://playerstage.sourceforge.net/gazebo/xmlschema/#gz' xmlns:model='http://playerstage.sourceforge.net/gazebo/xmlschema/#model' xmlns:sensor='http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor' xmlns:body='http://playerstage.sourceforge.net/gazebo/xmlschema/#body' xmlns:geom='http://playerstage.sourceforge.net/gazebo/xmlschema/#geom' xmlns:joint='http://playerstage.sourceforge.net/gazebo/xmlschema/#joint' xmlns:interface='http://playerstage.sourceforge.net/gazebo/xmlschema/#interface' xmlns:rendering='http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering' xmlns:renderable='http://playerstage.sourceforge.net/gazebo/xmlschema/#renderable' xmlns:controller='http://playerstage.sourceforge.net/gazebo/xmlschema/#controller' xmlns:physics='http://playerstage.sourceforge.net/gazebo/xmlschema/#physics' >");
       this->xmlSuffix_ = std::string("</gazebo:world>");
 
       // reset topic connection counts
       this->pub_link_states_connection_count_ = 0;
       this->pub_model_states_connection_count_ = 0;
       this->lastWrenchBodyUpdateTime = 0;
       this->lastForceJointUpdateTime = 0;
     }
 
     ~GazeboROSNode()
     {
       // disconnect slots
       gazebo::World::Instance()->DisconnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::wrenchBodySchedulerSlot,this));
       gazebo::World::Instance()->DisconnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::forceJointSchedulerSlot,this));
       gazebo::World::Instance()->DisconnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::publishSimTime,this));
       if (this->pub_link_states_connection_count_ > 0) // disconnect if there are subscribers on exit
         gazebo::World::Instance()->DisconnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::publishLinkStates,this));
       if (this->pub_model_states_connection_count_ > 0) // disconnect if there are subscribers on exit
         gazebo::World::Instance()->DisconnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::publishModelStates,this));
 
       // shutdown ros
       this->rosnode_.shutdown();
 
       // shutdown ros queue
       this->gazebo_callback_queue_thread_->join();
       delete this->gazebo_callback_queue_thread_;
 
       this->physics_reconfigure_thread_->join();
       delete this->physics_reconfigure_thread_;
 
       this->lock_.lock();
       for (std::vector<GazeboROSNode::ForceJointJob*>::iterator iter=this->force_joint_jobs.begin();iter!=this->force_joint_jobs.end();)
       {
         delete (*iter);
         this->force_joint_jobs.erase(iter);
       }
       for (std::vector<GazeboROSNode::WrenchBodyJob*>::iterator iter=this->wrench_body_jobs.begin();iter!=this->wrench_body_jobs.end();)
       {
         delete (*iter);
         this->wrench_body_jobs.erase(iter);
       }
       this->lock_.unlock();
     }
 
     void gazeboQueueThread()
     {
       ROS_DEBUG_STREAM("Callback thread id=" << boost::this_thread::get_id());
       static const double timeout = 0.001;
       static const double status_timeout = 1.0;
 
       ros::WallTime last_status_time;
       std_msgs::Bool stat;
       while (this->rosnode_.ok())
       {
         this->gazebo_queue_.callAvailable(ros::WallDuration(timeout));
 
         if ((ros::WallTime::now() - last_status_time).toSec() > status_timeout)
         {
                 stat.data = gazebo::Simulator::Instance()->IsPaused();
                 pub_gazebo_paused_.publish(stat);
                 last_status_time = ros::WallTime::now();
         }
       }
     }
 
     void AdvertiseServices()
     {
       // Advertise spawn services on the custom queue
       std::string spawn_urdf_model_service_name("spawn_urdf_model");
       ros::AdvertiseServiceOptions spawn_urdf_model_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
           spawn_urdf_model_service_name,boost::bind(&GazeboROSNode::spawnURDFModel,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       spawn_urdf_model_service_ = this->rosnode_.advertiseService(spawn_urdf_model_aso);
 
       // Advertise spawn services on the custom queue
       std::string spawn_gazebo_model_service_name("spawn_gazebo_model");
       ros::AdvertiseServiceOptions spawn_gazebo_model_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::SpawnModel>(
           spawn_gazebo_model_service_name,boost::bind(&GazeboROSNode::spawnGazeboModel,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       spawn_urdf_gazebo_service_ = this->rosnode_.advertiseService(spawn_gazebo_model_aso);
 
       // Advertise delete services on the custom queue
       std::string delete_model_service_name("delete_model");
       ros::AdvertiseServiceOptions delete_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::DeleteModel>(
           delete_model_service_name,boost::bind(&GazeboROSNode::deleteModel,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       delete_model_service_ = this->rosnode_.advertiseService(delete_aso);
 
       // Advertise more services on the custom queue
       std::string get_model_properties_service_name("get_model_properties");
       ros::AdvertiseServiceOptions get_model_properties_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelProperties>(
           get_model_properties_service_name,boost::bind(&GazeboROSNode::getModelProperties,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       get_model_properties_service_ = this->rosnode_.advertiseService(get_model_properties_aso);
 
       // Advertise more services on the custom queue
       std::string get_model_state_service_name("get_model_state");
       ros::AdvertiseServiceOptions get_model_state_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::GetModelState>(
           get_model_state_service_name,boost::bind(&GazeboROSNode::getModelState,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       get_model_state_service_ = this->rosnode_.advertiseService(get_model_state_aso);
 
       // Advertise more services on the custom queue
       std::string get_world_properties_service_name("get_world_properties");
       ros::AdvertiseServiceOptions get_world_properties_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::GetWorldProperties>(
           get_world_properties_service_name,boost::bind(&GazeboROSNode::getWorldProperties,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       get_world_properties_service_ = this->rosnode_.advertiseService(get_world_properties_aso);
 
       // Advertise more services on the custom queue
       std::string get_joint_properties_service_name("get_joint_properties");
       ros::AdvertiseServiceOptions get_joint_properties_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::GetJointProperties>(
           get_joint_properties_service_name,boost::bind(&GazeboROSNode::getJointProperties,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       get_joint_properties_service_ = this->rosnode_.advertiseService(get_joint_properties_aso);
 
       // Advertise more services on the custom queue
       std::string get_link_properties_service_name("get_link_properties");
       ros::AdvertiseServiceOptions get_link_properties_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLinkProperties>(
           get_link_properties_service_name,boost::bind(&GazeboROSNode::getLinkProperties,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       get_link_properties_service_ = this->rosnode_.advertiseService(get_link_properties_aso);
 
       // Advertise more services on the custom queue
       std::string get_link_state_service_name("get_link_state");
       ros::AdvertiseServiceOptions get_link_state_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::GetLinkState>(
           get_link_state_service_name,boost::bind(&GazeboROSNode::getLinkState,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       get_link_state_service_ = this->rosnode_.advertiseService(get_link_state_aso);
 
       // Advertise more services on the custom queue
       std::string set_link_properties_service_name("set_link_properties");
       ros::AdvertiseServiceOptions set_link_properties_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLinkProperties>(
           set_link_properties_service_name,boost::bind(&GazeboROSNode::setLinkProperties,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       set_link_properties_service_ = this->rosnode_.advertiseService(set_link_properties_aso);
 
       // Advertise more services on the custom queue
       std::string set_physics_properties_service_name("set_physics_properties");
       ros::AdvertiseServiceOptions set_physics_properties_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::SetPhysicsProperties>(
           set_physics_properties_service_name,boost::bind(&GazeboROSNode::setPhysicsProperties,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       set_physics_properties_service_ = this->rosnode_.advertiseService(set_physics_properties_aso);
 
       // Advertise more services on the custom queue
       std::string get_physics_properties_service_name("get_physics_properties");
       ros::AdvertiseServiceOptions get_physics_properties_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::GetPhysicsProperties>(
           get_physics_properties_service_name,boost::bind(&GazeboROSNode::getPhysicsProperties,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       get_physics_properties_service_ = this->rosnode_.advertiseService(get_physics_properties_aso);
 
       // Advertise more services on the custom queue
       std::string apply_body_wrench_service_name("apply_body_wrench");
       ros::AdvertiseServiceOptions apply_body_wrench_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::ApplyBodyWrench>(
           apply_body_wrench_service_name,boost::bind(&GazeboROSNode::applyBodyWrench,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       apply_body_wrench_service_ = this->rosnode_.advertiseService(apply_body_wrench_aso);
 
       // Advertise more services on the custom queue
       std::string set_model_state_service_name("set_model_state");
       ros::AdvertiseServiceOptions set_model_state_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::SetModelState>(
           set_model_state_service_name,boost::bind(&GazeboROSNode::setModelState,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       set_model_state_service_ = this->rosnode_.advertiseService(set_model_state_aso);
 
       // Advertise more services on the custom queue
       std::string apply_joint_effort_service_name("apply_joint_effort");
       ros::AdvertiseServiceOptions apply_joint_effort_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::ApplyJointEffort>(
           apply_joint_effort_service_name,boost::bind(&GazeboROSNode::applyJointEffort,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       apply_joint_effort_service_ = this->rosnode_.advertiseService(apply_joint_effort_aso);
 
       // Advertise more services on the custom queue
       std::string set_joint_properties_service_name("set_joint_properties");
       ros::AdvertiseServiceOptions set_joint_properties_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::SetJointProperties>(
           set_joint_properties_service_name,boost::bind(&GazeboROSNode::setJointProperties,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       set_joint_properties_service_ = this->rosnode_.advertiseService(set_joint_properties_aso);
 
       // Advertise more services on the custom queue
       std::string set_model_configuration_service_name("set_model_configuration");
       ros::AdvertiseServiceOptions set_model_configuration_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::SetModelConfiguration>(
           set_model_configuration_service_name,boost::bind(&GazeboROSNode::setModelConfiguration,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       set_model_configuration_service_ = this->rosnode_.advertiseService(set_model_configuration_aso);
 
       // Advertise more services on the custom queue
       std::string set_link_state_service_name("set_link_state");
       ros::AdvertiseServiceOptions set_link_state_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::SetLinkState>(
           set_link_state_service_name,boost::bind(&GazeboROSNode::setLinkState,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       set_link_state_service_ = this->rosnode_.advertiseService(set_link_state_aso);
 
       // Advertise more services on the custom queue
       std::string reset_simulation_service_name("reset_simulation");
       ros::AdvertiseServiceOptions reset_simulation_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
           reset_simulation_service_name,boost::bind(&GazeboROSNode::resetSimulation,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       reset_simulation_service_ = this->rosnode_.advertiseService(reset_simulation_aso);
 
       // Advertise more services on the custom queue
       std::string reset_world_service_name("reset_world");
       ros::AdvertiseServiceOptions reset_world_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
           reset_world_service_name,boost::bind(&GazeboROSNode::resetWorld,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       reset_world_service_ = this->rosnode_.advertiseService(reset_world_aso);
 
       // Advertise more services on the custom queue
       std::string pause_physics_service_name("pause_physics");
       ros::AdvertiseServiceOptions pause_physics_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
           pause_physics_service_name,boost::bind(&GazeboROSNode::pausePhysics,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       pause_physics_service_ = this->rosnode_.advertiseService(pause_physics_aso);
 
       // Advertise more services on the custom queue
       std::string unpause_physics_service_name("unpause_physics");
       ros::AdvertiseServiceOptions unpause_physics_aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
           unpause_physics_service_name,boost::bind(&GazeboROSNode::unpausePhysics,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       unpause_physics_service_ = this->rosnode_.advertiseService(unpause_physics_aso);
 
       // Advertise more services on the custom queue
       std::string clear_joint_forces_service_name("clear_joint_forces");
       ros::AdvertiseServiceOptions clear_joint_forces_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::JointRequest>(
           clear_joint_forces_service_name,boost::bind(&GazeboROSNode::clearJointForces,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       clear_joint_forces_service_ = this->rosnode_.advertiseService(clear_joint_forces_aso);
 
       // Advertise more services on the custom queue
       std::string clear_body_wrenches_service_name("clear_body_wrenches");
       ros::AdvertiseServiceOptions clear_body_wrenches_aso = ros::AdvertiseServiceOptions::create<gazebo_msgs::BodyRequest>(
           clear_body_wrenches_service_name,boost::bind(&GazeboROSNode::clearBodyWrenches,this,_1,_2),
           ros::VoidPtr(), &this->gazebo_queue_);
       clear_body_wrenches_service_ = this->rosnode_.advertiseService(clear_body_wrenches_aso);
 
       // Advertise topic on custom queue
       // topic callback version for set_link_state
       ros::SubscribeOptions link_state_so = ros::SubscribeOptions::create<gazebo_msgs::LinkState>(
         "set_link_state",10, boost::bind( &GazeboROSNode::updateLinkState,this,_1),
         ros::VoidPtr(), &this->gazebo_queue_);
       set_link_state_topic_ = this->rosnode_.subscribe(link_state_so);
 
       // topic callback version for set_model_state
       ros::SubscribeOptions model_state_so = ros::SubscribeOptions::create<gazebo_msgs::ModelState>(
         "set_model_state",10, boost::bind( &GazeboROSNode::updateModelState,this,_1),
         ros::VoidPtr(), &this->gazebo_queue_);
       set_model_state_topic_ = this->rosnode_.subscribe(model_state_so);
 
       // publish clock for simulated ros time
       pub_clock_ = this->rosnode_.advertise<rosgraph_msgs::Clock>("/clock",10);
 
       // publish complete link states in world frame
       ros::AdvertiseOptions pub_link_states_ao = ros::AdvertiseOptions::create<gazebo_msgs::LinkStates>(
         "link_states",10,
         boost::bind(&GazeboROSNode::onLinkStatesConnect,this),
         boost::bind(&GazeboROSNode::onLinkStatesDisconnect,this), ros::VoidPtr(), &this->gazebo_queue_);
       pub_link_states_ = this->rosnode_.advertise(pub_link_states_ao);
 
       // publish complete model states in world frame
       ros::AdvertiseOptions pub_model_states_ao = ros::AdvertiseOptions::create<gazebo_msgs::ModelStates>(
         "model_states",10,
         boost::bind(&GazeboROSNode::onModelStatesConnect,this),
         boost::bind(&GazeboROSNode::onModelStatesDisconnect,this), ros::VoidPtr(), &this->gazebo_queue_);
       pub_model_states_ = this->rosnode_.advertise(pub_model_states_ao);
 
 
 
 
 
       // set param for use_sim_time if not set by user alread
       this->rosnode_.setParam("/use_sim_time", true);
 
       // todo: contemplate setting environment variable ROBOT=sim here???
     }
 
     void onLinkStatesConnect()
     {
       this->pub_link_states_connection_count_++;
       if (this->pub_link_states_connection_count_ == 1) // connect on first subscriber
         gazebo::World::Instance()->ConnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::publishLinkStates,this));
     }
     void onModelStatesConnect()
     {
       this->pub_model_states_connection_count_++;
       if (this->pub_model_states_connection_count_ == 1) // connect on first subscriber
         gazebo::World::Instance()->ConnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::publishModelStates,this));
     }
     void onLinkStatesDisconnect()
     {
       this->pub_link_states_connection_count_--;
       if (this->pub_link_states_connection_count_ == 0) // disconnect with no subscribers
         gazebo::World::Instance()->DisconnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::publishLinkStates,this));
       else if (this->pub_link_states_connection_count_ < 0) // should not be possible
         ROS_ERROR("one too mandy disconnect from pub_link_states_ in gazeboros.cpp? something weird");
     }
     void onModelStatesDisconnect()
     {
       this->pub_model_states_connection_count_--;
       if (this->pub_model_states_connection_count_ == 0) // disconnect with no subscribers
         gazebo::World::Instance()->DisconnectWorldUpdateStartSignal(boost::bind(&GazeboROSNode::publishModelStates,this));
       else if (this->pub_model_states_connection_count_ < 0) // should not be possible
         ROS_ERROR("one too mandy disconnect from pub_model_states_ in gazeboros.cpp? something weird");
     }
 
     bool spawnURDFModel(gazebo_msgs::SpawnModel::Request &req,gazebo_msgs::SpawnModel::Response &res)
     {
       // get flag on joint limits
       bool enforce_limits = true; //option to add req.enforce_limits??
 
       // get name space for the corresponding model plugins
       std::string robot_namespace = req.robot_namespace;
 
       // incoming robot name
       std::string model_name = req.model_name;
 
       // incoming robot model string
       std::string model_xml = req.model_xml;
 
       if (!this->IsURDF(model_xml) && !IsColladaData(model_xml))
       {
         ROS_ERROR("SpawnModel: Failure - model format is not URDF (nor COLLADA).");
         res.success = false;
         res.status_message = "SpawnModel: Failure - model format is not URDF (nor COLLADA).";
         return false;
       }
 
       std::string open_bracket("<?");
       std::string close_bracket("?>");
       size_t pos1 = model_xml.find(open_bracket,0);
       size_t pos2 = model_xml.find(close_bracket,0);
       if (pos1 != std::string::npos && pos2 != std::string::npos)
         model_xml.replace(pos1,pos2-pos1+2,std::string(""));
 
       ROS_DEBUG("Model XML\n\n%s\n\n ",model_xml.c_str());
 
       urdf2gazebo::URDF2Gazebo u2g;
       TiXmlDocument model_xml_doc;
       model_xml_doc.Parse(model_xml.c_str());
       TiXmlDocument gazebo_model_xml;
       // initial xyz and rpy are empty, will be inserted in spawnGazeboModel
       u2g.convert(model_xml_doc, gazebo_model_xml, enforce_limits, urdf::Vector3(), urdf::Vector3(), model_name, robot_namespace);
 
       // push to factory iface
       std::ostringstream stream;
       stream << gazebo_model_xml;
       std::string gazebo_model_xml_string = stream.str();
       ROS_DEBUG("Gazebo Model XML\n\n%s\n\n ",gazebo_model_xml_string.c_str());
       req.model_xml = gazebo_model_xml_string;
 
       // publish gazebo model on parameter server as [robot_namespace]/gazebo/[model name]/robot_description
       // this->rosnode_.setParam(robot_namespace+std::string("/gazebo/")+model_name+std::string("/robot_description"),gazebo_model_xml_string);
 
       return spawnGazeboModel(req,res);
     }
 
     bool spawnGazeboModel(gazebo_msgs::SpawnModel::Request &req,gazebo_msgs::SpawnModel::Response &res)
     {
       // check to see if model name already exist as a model
       std::string model_name = req.model_name;
       if (gazebo::World::Instance()->GetEntityByName(model_name))
       {
         ROS_ERROR("SpawnModel: Failure - model name %s already exist.",model_name.c_str());
         res.success = false;
         res.status_message = "SpawnModel: Failure - model already exists.";
         return false;
       }
 
       // get name space for the corresponding model plugins
       std::string robot_namespace = req.robot_namespace;
 
       // get initial pose of model
       gazebo::Vector3 initial_xyz(req.initial_pose.position.x,req.initial_pose.position.y,req.initial_pose.position.z);
       // get initial roll pitch yaw (fixed frame transform)
       gazebo::Quatern initial_q(req.initial_pose.orientation.w,req.initial_pose.orientation.x,req.initial_pose.orientation.y,req.initial_pose.orientation.z);
 
       // refernce frame for initial pose definition, modify initial pose if defined
       gazebo::Body* frame = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(req.reference_frame));
       if (frame)
       {
         // convert to relative pose
         gazebo::Pose3d frame_pose = frame->GetWorldPose();
         initial_xyz = frame_pose.rot.RotateVector(initial_xyz);
         initial_xyz += frame_pose.pos;
         initial_q = frame_pose.rot*initial_q;
       }
 
       else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
       {
           ROS_DEBUG("SpawnModel: reference_frame is empty/world/map, using inertial frame");
       }
       else
       {
         res.success = false;
         res.status_message = "SpawnModel: reference reference_frame not found, did you forget to scope the link by model name?";
         return false;
       }
 
       // incoming robot model string
       std::string model_xml = req.model_xml;
 
       // store resulting Gazebo Model XML to be sent to spawn queue
       // get incoming string containg either an URDF or a Gazebo Model XML
       // grab from parameter server if necessary
       // convert to Gazebo Model XML if necessary
       if (!this->IsGazeboModelXML(model_xml))
       {
         ROS_ERROR("GazeboROSNode SpawnModel Failure: input model_xml not Gazebo XML, or cannot be converted to Gazebo XML");
         res.success = false;
         res.status_message = std::string("GazeboROSNode SpawnModel Failure: input model_xml not Gazebo XML, or cannot be converted to Gazebo XML");
         return false;
       }
 
       TiXmlDocument gazebo_model_xml;
 
       // incoming robot model string is a string containing a Gazebo Model XML
       std::string open_bracket("<?");
       std::string close_bracket("?>");
       size_t pos1 = model_xml.find(open_bracket,0);
       size_t pos2 = model_xml.find(close_bracket,0);
       if (pos1 != std::string::npos && pos2 != std::string::npos)
         model_xml.replace(pos1,pos2-pos1+2,std::string(""));
 
       // put string in TiXmlDocument for manipulation
       gazebo_model_xml.Parse(model_xml.c_str());
 
       // optional model manipulations:
       //  * update initial pose
       //  * replace model name
       TiXmlElement* model;
       model = gazebo_model_xml.FirstChildElement("model:physical");
       if (model)
       {
         // replace initial pose of robot
         // find first instance of xyz and rpy, replace with initial pose
         TiXmlElement* xyz_key = model->FirstChildElement("xyz");
         if (xyz_key)
           model->RemoveChild(xyz_key);
         TiXmlElement* rpy_key = model->FirstChildElement("rpy");
         if (rpy_key)
           model->RemoveChild(rpy_key);
 
         xyz_key = new TiXmlElement("xyz");
         rpy_key = new TiXmlElement("rpy");
 
         std::ostringstream xyz_stream, rpy_stream;
         xyz_stream << initial_xyz.x << " " << initial_xyz.y << " " << initial_xyz.z;
         gazebo::Vector3 initial_rpy = initial_q.GetAsEuler(); // convert to Euler angles for Gazebo XML
         rpy_stream << initial_rpy.x*180.0/M_PI << " " << initial_rpy.y*180.0/M_PI << " " << initial_rpy.z*180.0/M_PI; // convert to degrees for Gazebo (though ROS is in Radians)
 
 
         TiXmlText* xyz_txt = new TiXmlText(xyz_stream.str());
         TiXmlText* rpy_txt = new TiXmlText(rpy_stream.str());
 
         xyz_key->LinkEndChild(xyz_txt);
         rpy_key->LinkEndChild(rpy_txt);
 
         model->LinkEndChild(xyz_key);
         model->LinkEndChild(rpy_key);
 
         // replace model name if one is specified by the user
         model->RemoveAttribute("name");
         model->SetAttribute("name",model_name);
 
       }
 
       // push to factory iface
       std::ostringstream stream;
       stream << gazebo_model_xml;
       std::string gazebo_model_xml_string = stream.str();
       ROS_DEBUG("Gazebo Model XML\n\n%s\n\n ",gazebo_model_xml_string.c_str());
 
       // Add the new models into the World
       std::string xmlMiddle = gazebo_model_xml_string;
       // Strip leading <?xml...?> tag, if present, to allow the client to
       // pass the contents of a valid .model file
       std::string xmlVersion = "<?xml version=\"1.0\"?>";
       int i = xmlMiddle.find(xmlVersion);
       if(i >= 0) xmlMiddle.replace(i, xmlVersion.length(), "");
       // insert entity into a queue for gazebo to process
       std::string xmlString = this->xmlPrefix_ + xmlMiddle + this->xmlSuffix_;
       {
         boost::recursive_mutex::scoped_lock mrlock(*gazebo::Simulator::Instance()->GetMRMutex());
         gazebo::World::Instance()->InsertEntity(xmlString);
       }
 
       // process spawning
       //gazebo::World::Instance()->ProcessEntitiesToLoad();
 
       ros::Duration model_spawn_timeout(60.0);
       ros::Time timeout = ros::Time::now() + model_spawn_timeout;
       while (true)
       {
         if (ros::Time::now() > timeout)
         {
           res.success = false;
           res.status_message = std::string("SpawnModel: Model pushed to spawn queue, but spawn service timed out waiting for model to appear in simulation");
           return false;
         }
         {
           boost::recursive_mutex::scoped_lock lock(*gazebo::Simulator::Instance()->GetMRMutex());
           if (gazebo::World::Instance()->GetEntityByName(model_name)) break;
         }
         ROS_DEBUG("Waiting for spawning model (%s)",model_name.c_str());
         usleep(1000);
       }
 
       // set result
       res.success = true;
       res.status_message = std::string("SpawnModel: successfully spawned model");
       return true;
 
     }
 
     bool deleteModel(gazebo_msgs::DeleteModel::Request &req,gazebo_msgs::DeleteModel::Response &res)
     {
       // clear forces, etc for the body in question
 
       gazebo::Model* model = dynamic_cast<gazebo::Model*>(gazebo::World::Instance()->GetEntityByName(req.model_name));
       if (!model)
       {
         ROS_ERROR("DeleteModel: model [%s] does not exist",req.model_name.c_str());
         res.success = false;
         res.status_message = "DeleteModel: model does not exist";
         return false;
       }
 
       // delete wrench jobs on bodies
       const std::vector<gazebo::Entity*> children = model->GetChildren();
       for (std::vector<gazebo::Entity*>::const_iterator iter=children.begin();iter!=children.end();iter++)
       {
         gazebo::Body* body = (gazebo::Body*)(*iter);
         if (body)
         {
           // look for it in jobs, delete body wrench jobs
           this->clearBodyWrenches(body->GetScopedName());
         }
       }
       // delete force jobs on joints
       for (unsigned int i=0;i< model->GetJointCount(); i++)
       {
         gazebo::Joint* joint = model->GetJoint(i);
         // look for it in jobs, delete joint force jobs
         this->clearJointForces(joint->GetName());
       }
 
       // delete model
       gazebo::World::Instance()->SetSelectedEntity(NULL);
       gazebo::World::Instance()->DeleteEntity(req.model_name);
 
       ros::Duration model_spawn_timeout(60.0);
       ros::Time timeout = ros::Time::now() + model_spawn_timeout;
       // wait and verify that model is deleted
       while (true)
       {
         if (ros::Time::now() > timeout)
         {
           res.success = false;
           res.status_message = std::string("DeleteModel: Model pushed to delete queue, but delete service timed out waiting for model to disappear from simulation");
           return false;
         }
         {
           boost::recursive_mutex::scoped_lock lock(*gazebo::Simulator::Instance()->GetMRMutex());
           if (!gazebo::World::Instance()->GetEntityByName(req.model_name)) break;
         }
         ROS_DEBUG("Waiting for model deletion (%s)",req.model_name.c_str());
         usleep(1000);
       }
 
       // set result
       res.success = true;
       res.status_message = std::string("DeleteModel: successfully deleted model");
       return true;
     }
 
     bool getModelState(gazebo_msgs::GetModelState::Request &req,gazebo_msgs::GetModelState::Response &res)
     {
       gazebo::Model* model = dynamic_cast<gazebo::Model*>(gazebo::World::Instance()->GetEntityByName(req.model_name));
       gazebo::Body* frame = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(req.relative_entity_name));
       if (!model)
       {
         ROS_ERROR("GetModelState: model [%s] does not exist",req.model_name.c_str());
         res.success = false;
         res.status_message = "GetModelState: model does not exist";
         return false;
       }
       else
       {
         // get model pose
         gazebo::Pose3d  model_pose = model->GetWorldPose(); // - this->myBody->GetCoMPose();
         gazebo::Vector3 model_pos = model_pose.pos;
         gazebo::Quatern model_rot = model_pose.rot;
 
         // get model twist
         gazebo::Vector3 model_linear_vel  = model->GetWorldLinearVel();
         gazebo::Vector3 model_angular_vel = model->GetWorldAngularVel();
 
 
         if (frame)
         {
           // convert to relative pose
           gazebo::Pose3d frame_pose = frame->GetWorldPose();
           model_pos = model_pos - frame_pose.pos;
           model_pos = frame_pose.rot.RotateVectorReverse(model_pos);
           model_rot *= frame_pose.rot.GetInverse();
 
           // convert to relative rates
           gazebo::Vector3 frame_vpos = frame->GetWorldLinearVel(); // get velocity in gazebo frame
           gazebo::Vector3 frame_veul = frame->GetWorldAngularVel(); // get velocity in gazebo frame
           model_linear_vel = frame_pose.rot.RotateVector(model_linear_vel - frame_vpos);
           model_angular_vel = frame_pose.rot.RotateVector(model_angular_vel - frame_veul);
         }
         else if (req.relative_entity_name == "" || req.relative_entity_name == "world" || req.relative_entity_name == "map" || req.relative_entity_name == "/map")
         {
             ROS_DEBUG("GetModelState: relative_entity_name is empty/world/map, using inertial frame");
         }
         else
         {
           res.success = false;
           res.status_message = "GetModelState: reference relative_entity_name not found, did you forget to scope the body by model name?";
           return false;
         }
 
         // fill in response
         res.pose.position.x = model_pos.x;
         res.pose.position.y = model_pos.y;
         res.pose.position.z = model_pos.z;
         res.pose.orientation.w = model_rot.u;
         res.pose.orientation.x = model_rot.x;
         res.pose.orientation.y = model_rot.y;
         res.pose.orientation.z = model_rot.z;
 
         res.twist.linear.x = model_linear_vel.x;
         res.twist.linear.y = model_linear_vel.y;
         res.twist.linear.z = model_linear_vel.z;
         res.twist.angular.x = model_angular_vel.x;
         res.twist.angular.y = model_angular_vel.y;
         res.twist.angular.z = model_angular_vel.z;
 
         res.success = true;
         res.status_message = "GetModelState: got properties";
         return true;
       }
     }
 
     bool getModelProperties(gazebo_msgs::GetModelProperties::Request &req,gazebo_msgs::GetModelProperties::Response &res)
     {
       gazebo::Model* model = dynamic_cast<gazebo::Model*>(gazebo::World::Instance()->GetEntityByName(req.model_name));
       if (!model)
       {
         ROS_ERROR("GetModelProperties: model [%s] does not exist",req.model_name.c_str());
         res.success = false;
         res.status_message = "GetModelProperties: model does not exist";
         return false;
       }
       else
       {
         // get model parent name
         gazebo::Entity* parent_model = model->GetParent();
         if (parent_model) res.parent_model_name = parent_model->GetName();
 
         // get list of child bodies, geoms
         res.body_names.clear();
         res.geom_names.clear();
         const std::vector< gazebo::Entity* > entities = model->GetChildren();
         for (std::vector< gazebo::Entity* >::const_iterator eiter = entities.begin();eiter!=entities.end();eiter++)
         {
           gazebo::Body* body = dynamic_cast<gazebo::Body*>(*eiter);
           if (body)
           {
             res.body_names.push_back(body->GetName());
             // get list of geoms
             for (unsigned int i = 0; i < body->GetGeomCount() ; i++)
               res.geom_names.push_back(body->GetGeom(i)->GetName());
           }
         }
 
         // get list of joints
         res.joint_names.clear();
         unsigned int jc = model->GetJointCount();
         for (unsigned int i = 0; i < jc ; i++)
           res.joint_names.push_back( model->GetJoint(i)->GetName() );
 
         // get children model names
         res.child_model_names.clear();
         const std::vector< gazebo::Entity* > children = model->GetChildren();
         for (std::vector< gazebo::Entity* >::const_iterator citer = children.begin(); citer != children.end();  citer++)
           res.child_model_names.push_back((*citer)->GetName() );
 
         // is model static
         res.is_static = model->IsStatic();
 
         res.success = true;
         res.status_message = "GetModelProperties: got properties";
         return true;
       }
     }
 
     bool getWorldProperties(gazebo_msgs::GetWorldProperties::Request &req,gazebo_msgs::GetWorldProperties::Response &res)
     {
       res.sim_time = gazebo::Simulator::Instance()->GetSimTime().Double();
       res.model_names.clear();
       const std::vector<gazebo::Model*> models = gazebo::World::Instance()->GetModels();
       for (std::vector<gazebo::Model*>::const_iterator miter = models.begin(); miter != models.end(); miter++)
         res.model_names.push_back((*miter)->GetName());
       res.rendering_enabled = gazebo::Simulator::Instance()->GetRenderEngineEnabled();
       res.success = true;
       res.status_message = "GetWorldProperties: got properties";
 
       return true;
     }
 
     bool getJointProperties(gazebo_msgs::GetJointProperties::Request &req,gazebo_msgs::GetJointProperties::Response &res)
     {
       gazebo::Joint* joint = NULL;
       const std::vector<gazebo::Model*> models = gazebo::World::Instance()->GetModels();
       for (std::vector<gazebo::Model*>::const_iterator miter = models.begin(); miter != models.end(); miter++)
       {
         joint = (*miter)->GetJoint(req.joint_name);
         if (joint) break;
       }
 
       if (joint == NULL)
       {
         res.success = false;
         res.status_message = "GetJointProperties: joint not found";
         return false;
       }
       else
       {
         res.type = res.REVOLUTE;
 
         res.damping.clear(); // to be added to gazebo
         //res.damping.push_back(joint->GetDamping(0));
 
         res.position.clear(); // use GetAngle(i)
         res.position.push_back(joint->GetAngle(0).GetAsRadian());
 
         res.rate.clear(); // use GetVelocity(i)
         res.rate.push_back(joint->GetVelocity(0));
 
         res.success = true;
         res.status_message = "GetJointProperties: got properties";
         return true;
       }
     }
 
     bool getLinkProperties(gazebo_msgs::GetLinkProperties::Request &req,gazebo_msgs::GetLinkProperties::Response &res)
     {
       gazebo::Body* body = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(req.link_name));
       if (body == NULL)
       {
         res.success = false;
         res.status_message = "GetLinkProperties: link not found, did you forget to scope the link by model name?";
         return false;
       }
       else
       {
         res.gravity_mode = body->GetGravityMode();
 
         res.mass = body->GetMass().GetAsDouble();
 
         gazebo::Vector3 principalI = body->GetMass().GetPrincipalMoments();
         gazebo::Vector3 productI = body->GetMass().GetProductsofInertia();
         res.ixx = principalI.x;
         res.iyy = principalI.y;
         res.izz = principalI.z;
         res.ixy = productI.x;
         res.ixz = productI.y;
         res.iyz = productI.z;
 
         gazebo::Vector3 com = body->GetMass().GetCoG();
         res.com.position.x = com.x;
         res.com.position.y = com.y;
         res.com.position.z = com.z;
         res.com.orientation.x = 0; // @todo: gazebo do not support rotated inertia yet
         res.com.orientation.y = 0;
         res.com.orientation.z = 0;
         res.com.orientation.w = 1;
 
         res.success = true;
         res.status_message = "GetLinkProperties: got properties";
         return true;
       }
       return true;
     }
 
     bool getLinkState(gazebo_msgs::GetLinkState::Request &req,gazebo_msgs::GetLinkState::Response &res)
     {
       gazebo::Body* body = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(req.link_name));
       gazebo::Body* frame = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(req.reference_frame));
       if (body == NULL)
       {
         res.success = false;
         res.status_message = "GetLinkState: link not found, did you forget to scope the link by model name?";
         return false;
       }
 
       // get body pose
       gazebo::Pose3d body_pose = body->GetWorldPose();
       // Get inertial rates
       gazebo::Vector3 body_vpos = body->GetWorldLinearVel(); // get velocity in gazebo frame
       gazebo::Vector3 body_veul = body->GetWorldAngularVel(); // get velocity in gazebo frame
 
       if (frame)
       {
         // convert to relative pose
         gazebo::Pose3d frame_pose = frame->GetWorldPose();
         body_pose.pos = body_pose.pos - frame_pose.pos;
         body_pose.pos = frame_pose.rot.RotateVectorReverse(body_pose.pos);
         body_pose.rot *= frame_pose.rot.GetInverse();
 
         // convert to relative rates
         gazebo::Vector3 frame_vpos = frame->GetWorldLinearVel(); // get velocity in gazebo frame
         gazebo::Vector3 frame_veul = frame->GetWorldAngularVel(); // get velocity in gazebo frame
         body_vpos = frame_pose.rot.RotateVector(body_vpos - frame_vpos);
         body_veul = frame_pose.rot.RotateVector(body_veul - frame_veul);
       }
       else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
       {
           ROS_DEBUG("GetLinkState: reference_frame is empty/world/map, using inertial frame");
       }
       else
       {
         res.success = false;
         res.status_message = "GetLinkState: reference reference_frame not found, did you forget to scope the link by model name?";
         return false;
       }
 
       res.link_state.link_name = req.link_name;
       res.link_state.pose.position.x = body_pose.pos.x;
       res.link_state.pose.position.y = body_pose.pos.y;
       res.link_state.pose.position.z = body_pose.pos.z;
       res.link_state.pose.orientation.x = body_pose.rot.x;
       res.link_state.pose.orientation.y = body_pose.rot.y;
       res.link_state.pose.orientation.z = body_pose.rot.z;
       res.link_state.pose.orientation.w = body_pose.rot.u;
       res.link_state.twist.linear.x = body_vpos.x;
       res.link_state.twist.linear.y = body_vpos.y;
       res.link_state.twist.linear.z = body_vpos.z;
       res.link_state.twist.angular.x = body_veul.x;
       res.link_state.twist.angular.y = body_veul.y;
       res.link_state.twist.angular.z = body_veul.x;
       res.link_state.reference_frame = req.reference_frame;
 
       res.success = true;
       res.status_message = "GetLinkState: got state";
       return true;
     }
 
     bool setLinkProperties(gazebo_msgs::SetLinkProperties::Request &req,gazebo_msgs::SetLinkProperties::Response &res)
     {
       gazebo::Body* body = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(req.link_name));
       if (body == NULL)
       {
         res.success = false;
         res.status_message = "SetLinkProperties: link not found, did you forget to scope the link by model name?";
         return false;
       }
       else
       {
         gazebo::Mass mass;
         // @todo: FIXME: add inertia matrix rotation to Gazebo
         // mass.SetInertiaRotation(gazebo::Quaternion(req.com.orientation.w,res.com.orientation.x,req.com.orientation.y req.com.orientation.z));
         mass.SetCoG(gazebo::Vector3(req.com.position.x,req.com.position.y,req.com.position.z));
         mass.SetInertiaMatrix(req.ixx,req.iyy,req.izz,req.ixy,req.ixz,req.iyz);
         mass.SetMass(req.mass);
         body->SetMass(mass);
         body->SetGravityMode(req.gravity_mode);
         // @todo: mass change unverified
         res.success = true;
         res.status_message = "SetLinkProperties: properties set";
         return true;
       }
       return true;
     }
 
     bool setPhysicsProperties(gazebo_msgs::SetPhysicsProperties::Request &req,gazebo_msgs::SetPhysicsProperties::Response &res)
     {
       // pause simulation if requested
       bool is_paused = gazebo::Simulator::Instance()->IsPaused();
       gazebo::Simulator::Instance()->SetPaused(true);
 
       // supported updates
       gazebo::World::Instance()->GetPhysicsEngine()->SetStepTime(req.time_step);
       gazebo::World::Instance()->GetPhysicsEngine()->SetUpdateRate(req.max_update_rate);
       gazebo::World::Instance()->GetPhysicsEngine()->SetGravity(gazebo::Vector3(req.gravity.x,req.gravity.y,req.gravity.z));
 
       // stuff only works in ODE right now
       gazebo::World::Instance()->GetPhysicsEngine()->SetAutoDisableFlag(req.ode_config.auto_disable_bodies);
       gazebo::World::Instance()->GetPhysicsEngine()->SetSORPGSPreconIters(req.ode_config.sor_pgs_precon_iters);
       gazebo::World::Instance()->GetPhysicsEngine()->SetSORPGSIters(req.ode_config.sor_pgs_iters);
       gazebo::World::Instance()->GetPhysicsEngine()->SetRMSErrorTolerance(req.ode_config.sor_pgs_rms_error_tol);
       gazebo::World::Instance()->GetPhysicsEngine()->SetSORPGSW(req.ode_config.sor_pgs_w);
       gazebo::World::Instance()->GetPhysicsEngine()->SetContactSurfaceLayer(req.ode_config.contact_surface_layer);
       gazebo::World::Instance()->GetPhysicsEngine()->SetContactMaxCorrectingVel(req.ode_config.contact_max_correcting_vel);
       gazebo::World::Instance()->GetPhysicsEngine()->SetWorldCFM(req.ode_config.cfm);
       gazebo::World::Instance()->GetPhysicsEngine()->SetWorldERP(req.ode_config.erp);
       gazebo::World::Instance()->GetPhysicsEngine()->SetMaxContacts(req.ode_config.max_contacts);
 
 
       gazebo::Simulator::Instance()->SetPaused(is_paused);
       ROS_INFO("physics dynamics configure update complete");
 
       res.success = true;
       res.status_message = "physics engine updated";
       return true;
     }
 
     bool getPhysicsProperties(gazebo_msgs::GetPhysicsProperties::Request &req,gazebo_msgs::GetPhysicsProperties::Response &res)
     {
       // supported updates
       res.time_step = gazebo::World::Instance()->GetPhysicsEngine()->GetStepTime().Double();
       res.pause = gazebo::Simulator::Instance()->IsPaused();
       res.max_update_rate = gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate();
       gazebo::Vector3 gravity = gazebo::World::Instance()->GetPhysicsEngine()->GetGravity();
       res.gravity.x = gravity.x;
       res.gravity.y = gravity.y;
       res.gravity.z = gravity.z;
 
       // stuff only works in ODE right now
       res.ode_config.auto_disable_bodies = gazebo::World::Instance()->GetPhysicsEngine()->GetAutoDisableFlag();
       res.ode_config.sor_pgs_precon_iters = gazebo::World::Instance()->GetPhysicsEngine()->GetSORPGSPreconIters();
       res.ode_config.sor_pgs_iters = gazebo::World::Instance()->GetPhysicsEngine()->GetSORPGSIters();
       res.ode_config.sor_pgs_rms_error_tol = gazebo::World::Instance()->GetPhysicsEngine()->GetRMSErrorTolerance();
       res.ode_config.sor_pgs_w = gazebo::World::Instance()->GetPhysicsEngine()->GetSORPGSW();
       res.ode_config.contact_surface_layer = gazebo::World::Instance()->GetPhysicsEngine()->GetContactSurfaceLayer();
       res.ode_config.contact_max_correcting_vel = gazebo::World::Instance()->GetPhysicsEngine()->GetContactMaxCorrectingVel();
       res.ode_config.cfm = gazebo::World::Instance()->GetPhysicsEngine()->GetWorldCFM();
       res.ode_config.erp = gazebo::World::Instance()->GetPhysicsEngine()->GetWorldERP();
       res.ode_config.max_contacts = gazebo::World::Instance()->GetPhysicsEngine()->GetMaxContacts();
 
       res.success = true;
       res.status_message = "GetPhysicsProperties: got properties";
       return true;
     }
 
     bool setJointProperties(gazebo_msgs::SetJointProperties::Request &req,gazebo_msgs::SetJointProperties::Response &res)
     {
       gazebo::Joint* joint = NULL;
       const std::vector<gazebo::Model*> models = gazebo::World::Instance()->GetModels();
       for (std::vector<gazebo::Model*>::const_iterator miter = models.begin(); miter != models.end(); miter++)
       {
         joint = (*miter)->GetJoint(req.joint_name);
         if (joint) break;
       }
 
       if (joint == NULL)
       {
         res.success = false;
         res.status_message = "SetJointProperties: joint not found";
         return false;
       }
       else
       {
         for(unsigned int i=0;i< req.ode_joint_config.damping.size();i++)
           joint->SetDamping(i,req.ode_joint_config.damping[i]);
         for(unsigned int i=0;i< req.ode_joint_config.hiStop.size();i++)
           joint->SetAttribute(gazebo::Joint::HI_STOP,i,req.ode_joint_config.hiStop[i]);
         for(unsigned int i=0;i< req.ode_joint_config.loStop.size();i++)
           joint->SetAttribute(gazebo::Joint::LO_STOP,i,req.ode_joint_config.loStop[i]);
         for(unsigned int i=0;i< req.ode_joint_config.erp.size();i++)
           joint->SetAttribute(gazebo::Joint::ERP,i,req.ode_joint_config.erp[i]);
         for(unsigned int i=0;i< req.ode_joint_config.cfm.size();i++)
           joint->SetAttribute(gazebo::Joint::CFM,i,req.ode_joint_config.cfm[i]);
         for(unsigned int i=0;i< req.ode_joint_config.stop_erp.size();i++)
           joint->SetAttribute(gazebo::Joint::STOP_ERP,i,req.ode_joint_config.stop_erp[i]);
         for(unsigned int i=0;i< req.ode_joint_config.stop_cfm.size();i++)
           joint->SetAttribute(gazebo::Joint::STOP_CFM,i,req.ode_joint_config.stop_cfm[i]);
         for(unsigned int i=0;i< req.ode_joint_config.fudge_factor.size();i++)
           joint->SetAttribute(gazebo::Joint::FUDGE_FACTOR,i,req.ode_joint_config.fudge_factor[i]);
         for(unsigned int i=0;i< req.ode_joint_config.fmax.size();i++)
           joint->SetAttribute(gazebo::Joint::FMAX,i,req.ode_joint_config.fmax[i]);
         for(unsigned int i=0;i< req.ode_joint_config.vel.size();i++)
           joint->SetAttribute(gazebo::Joint::VEL,i,req.ode_joint_config.vel[i]);
 
         res.success = true;
         res.status_message = "SetJointProperties: properties set";
         return true;
       }
     }
 
     bool setModelState(gazebo_msgs::SetModelState::Request &req,gazebo_msgs::SetModelState::Response &res)
     {
       gazebo::Vector3 target_pos(req.model_state.pose.position.x,req.model_state.pose.position.y,req.model_state.pose.position.z);
       gazebo::Quatern target_rot(req.model_state.pose.orientation.w,req.model_state.pose.orientation.x,req.model_state.pose.orientation.y,req.model_state.pose.orientation.z);
       gazebo::Pose3d target_pose(target_pos,target_rot);
       gazebo::Vector3 target_pos_dot(req.model_state.twist.linear.x,req.model_state.twist.linear.y,req.model_state.twist.linear.z);
       gazebo::Vector3 target_rot_dot(req.model_state.twist.angular.x,req.model_state.twist.angular.y,req.model_state.twist.angular.z);
 
       gazebo::Model* model = dynamic_cast<gazebo::Model*>(gazebo::World::Instance()->GetEntityByName(req.model_state.model_name));
       if (!model)
       {
         ROS_ERROR("SetModelState: model [%s] does not exist",req.model_state.model_name.c_str());
         res.success = false;
         res.status_message = "SetModelState: model does not exist";
         return false;
       }
       else
       {
         gazebo::Entity* relative_entity = gazebo::World::Instance()->GetEntityByName(req.model_state.reference_frame);
         if (relative_entity)
         {
           gazebo::Pose3d  frame_pose = relative_entity->GetWorldPose(); // - this->myBody->GetCoMPose();
           gazebo::Vector3 frame_pos = frame_pose.pos;
           gazebo::Quatern frame_rot = frame_pose.rot;
 
           //std::cout << " debug : " << relative_entity->GetName() << " : " << frame_pose << " : " << target_pose << std::endl;
           //target_pose = frame_pose + target_pose; // seems buggy, use my own
           target_pose.pos = frame_pos + frame_rot.RotateVector(target_pos);
           target_pose.rot = frame_rot * target_pose.rot;
         }
         else if (req.model_state.reference_frame == "" || req.model_state.reference_frame == "world" || req.model_state.reference_frame == "map" || req.model_state.reference_frame == "/map" )
         {
           ROS_DEBUG("SetModelState: reference frame is empty/world/map, usig inertial frame");
         }
         else
         {
           ROS_ERROR("SetModelState: for model[%s], specified reference frame entity [%s] does not exist",
                     req.model_state.model_name.c_str(),req.model_state.reference_frame.c_str());
           res.success = false;
           res.status_message = "SetModelState: specified reference frame entity does not exist";
           return false;
         }
 
         //ROS_ERROR("target state: %f %f %f",target_pose.pos.x,target_pose.pos.y,target_pose.pos.z);
         bool is_paused = gazebo::Simulator::Instance()->IsPaused();
         gazebo::Simulator::Instance()->SetPaused(true);
         model->SetWorldPose(target_pose);
         gazebo::Simulator::Instance()->SetPaused(is_paused);
         //gazebo::Pose3d p3d = model->GetWorldPose();
         //ROS_ERROR("model updated state: %f %f %f",p3d.pos.x,p3d.pos.y,p3d.pos.z);
 
         // set model velocity
         model->SetLinearVel(target_pos_dot);
         model->SetAngularVel(target_rot_dot);
 
         res.success = true;
         res.status_message = "SetModelState: set model state done";
         return true;
       }
     }
 
     void updateModelState(const gazebo_msgs::ModelState::ConstPtr& model_state)
     {
       gazebo::Vector3 target_pos(model_state->pose.position.x,model_state->pose.position.y,model_state->pose.position.z);
       gazebo::Quatern target_rot(model_state->pose.orientation.w,model_state->pose.orientation.x,model_state->pose.orientation.y,model_state->pose.orientation.z);
       gazebo::Pose3d target_pose(target_pos,target_rot);
       gazebo::Vector3 target_pos_dot(model_state->twist.linear.x,model_state->twist.linear.y,model_state->twist.linear.z);
       gazebo::Vector3 target_rot_dot(model_state->twist.angular.x,model_state->twist.angular.y,model_state->twist.angular.z);
 
       gazebo::Model* model = dynamic_cast<gazebo::Model*>(gazebo::World::Instance()->GetEntityByName(model_state->model_name));
       if (!model)
       {
         ROS_ERROR("updateModelState: model [%s] does not exist",model_state->model_name.c_str());
         return;
       }
       else
       {
         gazebo::Entity* relative_entity = NULL;
 
         relative_entity = gazebo::World::Instance()->GetEntityByName(model_state->reference_frame);
         if (relative_entity)
         {
           gazebo::Pose3d  frame_pose = relative_entity->GetWorldPose(); // - this->myBody->GetCoMPose();
           gazebo::Vector3 frame_pos = frame_pose.pos;
           gazebo::Quatern frame_rot = frame_pose.rot;
 
           //std::cout << " debug : " << relative_entity->GetName() << " : " << frame_pose << " : " << target_pose << std::endl;
           //target_pose = frame_pose + target_pose; // seems buggy, use my own
           target_pose.pos = frame_pos + frame_rot.RotateVector(target_pos);
           target_pose.rot = frame_rot * target_pose.rot;
         }
         else if (model_state->reference_frame == "" || model_state->reference_frame == "world" || model_state->reference_frame == "map" || model_state->reference_frame == "/map" )
         {
           ROS_DEBUG("updateModelState: using inertial reference frame");
         }
         else
         {
           ROS_ERROR("updateModelState: for model[%s], specified relative frame [%s] does not exist",
                     model_state->model_name.c_str(),model_state->reference_frame.c_str());
           return;
         }
 
         bool is_paused = gazebo::Simulator::Instance()->IsPaused();
         gazebo::Simulator::Instance()->SetPaused(true);
         model->SetWorldPose(target_pose);
         gazebo::Simulator::Instance()->SetPaused(is_paused);
 
         // set model velocity to 0
         model->SetLinearVel(target_pos_dot);
         model->SetAngularVel(target_rot_dot);
 
         return;
       }
     }
 
     bool applyJointEffort(gazebo_msgs::ApplyJointEffort::Request &req,gazebo_msgs::ApplyJointEffort::Response &res)
     {
       gazebo::Joint* joint = NULL;
       const std::vector<gazebo::Model*> models = gazebo::World::Instance()->GetModels();
       for (std::vector<gazebo::Model*>::const_iterator miter = models.begin(); miter != models.end(); miter++)
       {
         joint = (*miter)->GetJoint(req.joint_name);
         if (joint) break;
       }
 
       if (joint == NULL)
       {
         res.success = false;
         res.status_message = "ApplyJointEffort: joint not found";
         return false;
       }
       else
       {
 
         GazeboROSNode::ForceJointJob* fjj = new GazeboROSNode::ForceJointJob;
         fjj->joint = joint;
         fjj->force = req.effort;
         fjj->start_time = req.start_time;
         if (fjj->start_time < ros::Time(gazebo::Simulator::Instance()->GetSimTime().Double()))
           fjj->start_time = ros::Time(gazebo::Simulator::Instance()->GetSimTime().Double());
         fjj->duration = req.duration;
         this->lock_.lock();
         this->force_joint_jobs.push_back(fjj);
         this->lock_.unlock();
 
         res.success = true;
         res.status_message = "ApplyJointEffort: effort set";
         return true;
       }
     }
 
     bool resetSimulation(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
     {
       gazebo::World::Instance()->Reset();
       return true;
     }
 
     bool resetWorld(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
     {
       gazebo::World::Instance()->Reset();
       return true;
     }
 
     bool pausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
     {
       gazebo::Simulator::Instance()->SetPaused(true);
       return true;
     }
 
     bool unpausePhysics(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
     {
       gazebo::Simulator::Instance()->SetPaused(false);
       return true;
     }
 
     bool clearJointForces(gazebo_msgs::JointRequest::Request &req,gazebo_msgs::JointRequest::Response &res)
     {
       return this->clearJointForces(req.joint_name);
     }
     bool clearJointForces(std::string joint_name)
     {
       bool search = true;
       this->lock_.lock();
       while(search)
       {
         search = false;
         for (std::vector<GazeboROSNode::ForceJointJob*>::iterator iter=this->force_joint_jobs.begin();iter!=this->force_joint_jobs.end();iter++)
         {
           if ((*iter)->joint->GetName() == joint_name)
           {
             // found one, search through again
             search = true;
             delete (*iter);
             this->force_joint_jobs.erase(iter);
             break;
           }
         }
       }
       this->lock_.unlock();
       return true;
     }
 
     bool clearBodyWrenches(gazebo_msgs::BodyRequest::Request &req,gazebo_msgs::BodyRequest::Response &res)
     {
       return this->clearBodyWrenches(req.body_name);
     }
     bool clearBodyWrenches(std::string body_name)
     {
       bool search = true;
       this->lock_.lock();
       while(search)
       {
         search = false;
         for (std::vector<GazeboROSNode::WrenchBodyJob*>::iterator iter=this->wrench_body_jobs.begin();iter!=this->wrench_body_jobs.end();iter++)
         {
           //ROS_ERROR("search %s %s",(*iter)->body->GetScopedName().c_str(), body_name.c_str());
           if ((*iter)->body->GetScopedName() == body_name)
           {
             // found one, search through again
             search = true;
             delete (*iter);
             this->wrench_body_jobs.erase(iter);
             break;
           }
         }
       }
       this->lock_.unlock();
       return true;
     }
 
     bool setModelConfiguration(gazebo_msgs::SetModelConfiguration::Request &req,gazebo_msgs::SetModelConfiguration::Response &res)
     {
       std::string gazebo_model_name = req.model_name;
 
       // search for model with name
       gazebo::Model* gazebo_model = dynamic_cast<gazebo::Model*>(gazebo::World::Instance()->GetEntityByName(gazebo_model_name));
       if (gazebo_model == NULL)
       {
         ROS_ERROR("SetModelConfiguration: model [%s] does not exist",gazebo_model_name.c_str());
         res.success = false;
         res.status_message = "SetModelConfiguration: model does not exist";
         return false;
       }
 
       // make the service call to pause gazebo
       bool is_paused = gazebo::Simulator::Instance()->IsPaused();
       if (!is_paused) gazebo::Simulator::Instance()->SetPaused(true);
 
       setModelJointPositions(gazebo_model,req.joint_names,req.joint_positions);
 
       // resume paused state before this call
       gazebo::Simulator::Instance()->SetPaused(is_paused);
 
       res.success = true;
       res.status_message = "SetModelConfiguration: success";
       return true;
     }
 
     void setModelJointPositions(gazebo::Model* gazebo_model,std::vector<std::string> joint_names, std::vector<double> joint_positions)
     {
 
       // go through all joints in this model and update each one
       //   for each joint update, recursively update all children
       int joint_count = gazebo_model->GetJointCount();
       for (int i = 0; i < joint_count ; i++)
       {
         gazebo::Joint* joint = gazebo_model->GetJoint(i);
 
         double target_position;
         // only deal with hinge and revolute joints in the user request joint_names list
         gazebo::Joint::Type type = joint->GetType();
         if ((type == gazebo::Joint::HINGE || type == gazebo::Joint::SLIDER)
             && findJointPosition(target_position,joint->GetName(),joint_names,joint_positions))
         {
           gazebo::Vector3 anchor = joint->GetAnchor(0); // worldPose.pos of anchor point
           gazebo::Vector3 axis = joint->GetAxis(0);     // joint axis in world frame
           double current_position = joint->GetAngle(0).GetAsRadian(); // joint position
           //std::cout << "    name[" << joint->GetName() << "] axis[" << axis << "] target [" << target_position << "]\n";
 
           gazebo::Body* body0 = getParentBody(joint);
           gazebo::Body* body1 = getChildBody(joint);
 
           if (body0 && body1 && body0->GetName() != body1->GetName())
           {
             
             // transform about the current anchor, about the axis
             switch(type)
             {
               case gazebo::Joint::HINGE: {
                 // rotate child (body1) about anchor point, by delta-angle along axis
                 double dangle = target_position - current_position;
                 rotateBodyAndChildren(body1, anchor, axis, dangle);
                 
                 break;
               }
               case gazebo::Joint::SLIDER: {
                 double dposition = target_position - current_position;
                 slideBodyAndChildren(body1, anchor, axis, dposition);
 
                 break;
               }
               default: {
                 ROS_WARN("Setting non HINGE/SLIDER joint types not implemented [%s]",joint->GetName().c_str());
                 break;
               }
             }
 
           }
 
         }
       }
     }
 
     gazebo::Body* getChildBody(gazebo::Joint* joint)
     {
       // anchor is with the child (same as urdf)
       if (joint->GetJointBody(0) == joint->anchorBody)
         return joint->GetJointBody(0);
       else if (joint->GetJointBody(1) == joint->anchorBody)
         return joint->GetJointBody(1);
       else
         return NULL;
     }
 
     gazebo::Body* getParentBody(gazebo::Joint* joint)
     {
       if (joint->GetJointBody(0) == joint->anchorBody)
         return joint->GetJointBody(1);
       else if (joint->GetJointBody(1) == joint->anchorBody)
         return joint->GetJointBody(0);
       else
         return NULL;
     }
 
     void rotateBodyAndChildren(gazebo::Body* body1, gazebo::Vector3 anchor, gazebo::Vector3 axis, double dangle, bool update_children = true)
     {
       gazebo::Pose3d world_pose = body1->GetWorldPose();
       gazebo::Pose3d relative_pose(world_pose.pos - anchor,world_pose.rot); // relative to anchor point
       // take axis rotation and turn it int a quaternion
       gazebo::Quatern rotation; rotation.SetFromAxis(axis.x,axis.y,axis.z,dangle);
       // rotate relative pose by rotation
       gazebo::Pose3d new_relative_pose;
       new_relative_pose.pos = rotation.RotateVector(relative_pose.pos);
       new_relative_pose.rot = rotation * relative_pose.rot;
       gazebo::Pose3d new_world_pose(new_relative_pose.pos+anchor,new_relative_pose.rot);
       body1->SetWorldPose(new_world_pose);
       // std::cout << "      body[" << body1->GetName()
       //           << "] wp[" << world_pose
       //           << "] anchor[" << anchor
       //           << "] axis[" << axis
       //           << "] dangle [" << dangle
       //           << "]\n";
 
       // recurse through children bodies
       std::vector<gazebo::Body*> bodies;
       if (update_children) getAllChildrenBodies(bodies, body1->GetModel(), body1);
       for (std::vector<gazebo::Body*>::iterator bit = bodies.begin(); bit != bodies.end(); bit++)
         rotateBodyAndChildren((*bit), anchor, axis, dangle,false);
 
     }
 
     void slideBodyAndChildren(gazebo::Body* body1, gazebo::Vector3 anchor, gazebo::Vector3 axis, double dposition, bool update_children = true)
     {
       gazebo::Pose3d world_pose = body1->GetWorldPose();
       gazebo::Pose3d relative_pose(world_pose.pos - anchor,world_pose.rot); // relative to anchor point
       // slide relative pose by dposition along axis
       gazebo::Pose3d new_relative_pose;
       new_relative_pose.pos = relative_pose.pos + axis * dposition;
       new_relative_pose.rot = relative_pose.rot;
       gazebo::Pose3d new_world_pose(new_relative_pose.pos+anchor,new_relative_pose.rot);
       body1->SetWorldPose(new_world_pose);
       // std::cout << "      body[" << body1->GetName()
       //           << "] wp[" << world_pose
       //           << "] anchor[" << anchor
       //           << "] axis[" << axis
       //           << "] dposition [" << dposition
       //           << "]\n";
 
       // recurse through children bodies
       std::vector<gazebo::Body*> bodies;
       if (update_children) getAllChildrenBodies(bodies, body1->GetModel(), body1);
       for (std::vector<gazebo::Body*>::iterator bit = bodies.begin(); bit != bodies.end(); bit++)
         slideBodyAndChildren((*bit), anchor, axis, dposition,false);
 
     }
 
     void getAllChildrenBodies(std::vector<gazebo::Body*> &bodies, gazebo::Model* model, gazebo::Body* body)
     {
       // strategy, for each child, recursively look for children
       //           for each child, also look for parents to catch multiple roots
       if (model)
       {
         int joint_count = model->GetJointCount();
         for (int i = 0; i < joint_count ; i++)
         {
           gazebo::Joint* joint = model->GetJoint(i);
           // recurse through children connected by joints
           gazebo::Body* body0 = getParentBody(joint);
           gazebo::Body* body1 = getChildBody(joint);
           if (body0 && body1
               && body0->GetName() != body1->GetName()
               && body0->GetName() == body->GetName()
               && !inBodies(body1,bodies))
           {
             bodies.push_back(body1);
             getAllChildrenBodies(bodies, body1->GetModel(), body1);
             getAllParentBodies(bodies, body1->GetModel(), body1, body);
           }
         }
       }
     }
     void getAllParentBodies(std::vector<gazebo::Body*> &bodies, gazebo::Model* model, gazebo::Body* body, gazebo::Body* orig_parent_body)
     {
       if (model)
       {
         int joint_count = model->GetJointCount();
         for (int i = 0; i < joint_count ; i++)
         {
           gazebo::Joint* joint = model->GetJoint(i);
           // recurse through children connected by joints
           gazebo::Body* body0 = getParentBody(joint);
           gazebo::Body* body1 = getChildBody(joint);
           if (body0 && body1
               && body0->GetName() != body1->GetName()
               && body1->GetName() == body->GetName()
               && body0->GetName() != orig_parent_body->GetName()
               && !inBodies(body0,bodies))
           {
             bodies.push_back(body0);
             getAllParentBodies(bodies, body0->GetModel(), body1, orig_parent_body);
           }
         }
       }
     }
 
     bool inBodies(gazebo::Body* body,std::vector<gazebo::Body*> bodies)
     {
       for (std::vector<gazebo::Body*>::iterator bit = bodies.begin(); bit != bodies.end(); bit++)
         if ((*bit)->GetName() == body->GetName()) return true;
       return false;
     }
 
     bool findJointPosition(double &position, std::string name, std::vector<std::string> joint_names, std::vector<double> joint_positions)
     {
       position = 0;
       std::vector<std::string>::iterator jit = joint_names.begin();
       std::vector<double>::iterator pit = joint_positions.begin();
       for(;jit != joint_names.end(),pit != joint_positions.end(); jit++,pit++)
       {
         if (name == (*jit))
         {
           position = (*pit);
           return true;
         }
       }
       return false;
     }
 
 
     bool setLinkState(gazebo_msgs::SetLinkState::Request &req,gazebo_msgs::SetLinkState::Response &res)
     {
       gazebo::Body* body = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(req.link_state.link_name));
       gazebo::Body* frame = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(req.link_state.reference_frame));
       if (!body)
       {
         ROS_ERROR("SetLinkState: link [%s] does not exist",req.link_state.link_name.c_str());
         res.success = false;
         res.status_message = "SetLinkState: link does not exist";
         return false;
       }
 
       // get reference frame (body/model(link)) pose and
       // transform target pose to absolute world frame
       gazebo::Vector3 target_pos(req.link_state.pose.position.x,req.link_state.pose.position.y,req.link_state.pose.position.z);
       gazebo::Quatern target_rot(req.link_state.pose.orientation.w,req.link_state.pose.orientation.x,req.link_state.pose.orientation.y,req.link_state.pose.orientation.z);
       gazebo::Pose3d target_pose(target_pos,target_rot);
       gazebo::Vector3 target_linear_vel(req.link_state.twist.linear.x,req.link_state.twist.linear.y,req.link_state.twist.linear.z);
       gazebo::Vector3 target_angular_vel(req.link_state.twist.angular.x,req.link_state.twist.angular.y,req.link_state.twist.angular.z);
 
       if (frame)
       {
         gazebo::Pose3d  frame_pose = frame->GetWorldPose(); // - this->myBody->GetCoMPose();
         gazebo::Vector3 frame_pos = frame_pose.pos;
         gazebo::Quatern frame_rot = frame_pose.rot;
 
         //std::cout << " debug : " << frame->GetName() << " : " << frame_pose << " : " << target_pose << std::endl;
         //target_pose = frame_pose + target_pose; // seems buggy, use my own
         target_pose.pos = frame_pos + frame_rot.RotateVector(target_pos);
         target_pose.rot = frame_rot * target_pose.rot;
 
         gazebo::Vector3 frame_linear_vel = frame->GetWorldLinearVel();
         gazebo::Vector3 frame_angular_vel = frame->GetWorldAngularVel();
         target_linear_vel -= frame_linear_vel;
         target_angular_vel -= frame_angular_vel;
       }
       else if (req.link_state.reference_frame == "" || req.link_state.reference_frame == "world" || req.link_state.reference_frame == "map" || req.link_state.reference_frame == "/map")
       {
         ROS_INFO("SetLinkState: reference_frame is empty/world/map, using inertial frame");
       }
       else
       {
         ROS_ERROR("SetLinkState: reference_frame is not a valid link name");
         res.success = false;
         res.status_message = "SetLinkState: failed";
         return false;
       }
       //target_pose.rot.Normalize();
 
       //std::cout << " debug : " << target_pose << std::endl;
       //boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
 
       bool is_paused = gazebo::Simulator::Instance()->IsPaused();
       if (!is_paused) gazebo::Simulator::Instance()->SetPaused(true);
       body->SetWorldPose(target_pose);
       gazebo::Simulator::Instance()->SetPaused(is_paused);
 
       // set body velocity to desired twist
       body->SetLinearVel(target_linear_vel);
       body->SetAngularVel(target_angular_vel);
 
       res.success = true;
       res.status_message = "SetLinkState: success";
       return true;
     }
 
     void updateLinkState(const gazebo_msgs::LinkState::ConstPtr& link_state)
     {
       gazebo::Body* body = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(link_state->link_name));
       gazebo::Body* frame = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(link_state->reference_frame));
       if (!body)
       {
         ROS_ERROR("SetLinkState: link [%s] does not exist",link_state->link_name.c_str());
         return;
       }
       else
       {
         // get reference frame (body/model(link)) pose and
         // transform target pose to absolute world frame
         gazebo::Vector3 target_pos(link_state->pose.position.x,link_state->pose.position.y,link_state->pose.position.z);
         gazebo::Quatern target_rot(link_state->pose.orientation.w,link_state->pose.orientation.x,link_state->pose.orientation.y,link_state->pose.orientation.z);
         gazebo::Pose3d target_pose(target_pos,target_rot);
         gazebo::Vector3 target_linear_vel(link_state->twist.linear.x,link_state->twist.linear.y,link_state->twist.linear.z);
         gazebo::Vector3 target_angular_vel(link_state->twist.angular.x,link_state->twist.angular.y,link_state->twist.angular.z);
 
         if (frame)
         {
           gazebo::Pose3d  frame_pose = frame->GetWorldPose(); // - this->myBody->GetCoMPose();
           gazebo::Vector3 frame_pos = frame_pose.pos;
           gazebo::Quatern frame_rot = frame_pose.rot;
 
           //std::cout << " debug : " << frame->GetName() << " : " << frame_pose << " : " << target_pose << std::endl;
           //target_pose = frame_pose + target_pose; // seems buggy, use my own
           target_pose.pos = frame_pos + frame_rot.RotateVector(target_pos);
           target_pose.rot = frame_rot * target_pose.rot;
 
           gazebo::Vector3 frame_linear_vel = frame->GetWorldLinearVel();
           gazebo::Vector3 frame_angular_vel = frame->GetWorldAngularVel();
           target_linear_vel -= frame_linear_vel;
           target_angular_vel -= frame_angular_vel;
         }
         else if (link_state->reference_frame == "" || link_state->reference_frame == "world" || link_state->reference_frame == "map" || link_state->reference_frame == "/map")
         {
           ROS_DEBUG("SetLinkState: reference_frame is empty/world/map, using inertial frame");
         }
         else
         {
           ROS_ERROR("SetLinkState: reference_frame is not a valid link name");
           return;
         }
 
         //std::cout << " debug : " << target_pose << std::endl;
         //boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
 
         bool is_paused = gazebo::Simulator::Instance()->IsPaused();
         gazebo::Simulator::Instance()->SetPaused(true);
         body->SetWorldPose(target_pose);
         gazebo::Simulator::Instance()->SetPaused(is_paused);
 
         // set body velocity to desired twist
         body->SetLinearVel(target_linear_vel);
         body->SetAngularVel(target_angular_vel);
 
       }
 
     }
 
 
     void transformWrench( gazebo::Vector3 &target_force, gazebo::Vector3 &target_torque,
                                gazebo::Vector3 reference_force, gazebo::Vector3 reference_torque,
                                gazebo::Pose3d target_to_reference )
     {
       // rotate force into target frame
       target_force = target_to_reference.rot.RotateVector(reference_force);
       // rotate torque into target frame
       target_torque = target_to_reference.rot.RotateVector(reference_torque);
 
       // target force is the refence force rotated by the target->reference transform
       target_force = target_force;
       target_torque = target_torque + target_to_reference.pos.GetCrossProd(target_force);
     }
 
     bool applyBodyWrench(gazebo_msgs::ApplyBodyWrench::Request &req,gazebo_msgs::ApplyBodyWrench::Response &res)
     {
       gazebo::Body* body = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(req.body_name));
       gazebo::Body* frame = dynamic_cast<gazebo::Body*>(gazebo::World::Instance()->GetEntityByName(req.reference_frame));
 
       if (!body)
       {
         ROS_ERROR("ApplyBodyWrench: body [%s] does not exist",req.body_name.c_str());
         res.success = false;
         res.status_message = "ApplyBodyWrench: body does not exist";
         return false;
       }
 
       // target wrench
       gazebo::Vector3 reference_force(req.wrench.force.x,req.wrench.force.y,req.wrench.force.z);
       gazebo::Vector3 reference_torque(req.wrench.torque.x,req.wrench.torque.y,req.wrench.torque.z);
       gazebo::Vector3 reference_point(req.reference_point.x,req.reference_point.y,req.reference_point.z);
 
       gazebo::Vector3 target_force;
       gazebo::Vector3 target_torque;
 
       reference_torque = reference_torque + reference_point.GetCrossProd(reference_force);
 
       if (frame)
       {
         // FIXME
         // get reference frame (body/model(body)) pose and
         // transform target pose to absolute world frame
         // @todo: need to modify wrench (target force and torque by frame)
         //        transform wrench from reference_point in reference_frame
         //        into the reference frame of the body
         //        first, translate by reference point to the body frame
         gazebo::Pose3d target_to_reference = frame->GetWorldPose() - body->GetWorldPose();
         ROS_DEBUG("reference frame for applied wrench: [%f %f %f, %f %f %f]-[%f %f %f, %f %f %f]=[%f %f %f, %f %f %f]",
                    body->GetWorldPose().pos.x,
                    body->GetWorldPose().pos.y,
                    body->GetWorldPose().pos.z,
                    body->GetWorldPose().rot.GetAsEuler().x,
                    body->GetWorldPose().rot.GetAsEuler().y,
                    body->GetWorldPose().rot.GetAsEuler().z,
                    frame->GetWorldPose().pos.x,
                    frame->GetWorldPose().pos.y,
                    frame->GetWorldPose().pos.z,
                    frame->GetWorldPose().rot.GetAsEuler().x,
                    frame->GetWorldPose().rot.GetAsEuler().y,
                    frame->GetWorldPose().rot.GetAsEuler().z,
                    target_to_reference.pos.x,
                    target_to_reference.pos.y,
                    target_to_reference.pos.z,
                    target_to_reference.rot.GetAsEuler().x,
                    target_to_reference.rot.GetAsEuler().y,
                    target_to_reference.rot.GetAsEuler().z
                  );
         this->transformWrench(target_force, target_torque, reference_force, reference_torque, target_to_reference);
         ROS_ERROR("wrench defined as [%s]:[%f %f %f, %f %f %f] --> applied as [%s]:[%f %f %f, %f %f %f]",
                    frame->GetName().c_str(),
                    reference_force.x,
                    reference_force.y,
                    reference_force.z,
                    reference_torque.x,
                    reference_torque.y,
                    reference_torque.z,
                    body->GetName().c_str(),
                    target_force.x,
                    target_force.y,
                    target_force.z,
                    target_torque.x,
                    target_torque.y,
                    target_torque.z
                  );
 
       }
       else if (req.reference_frame == "" || req.reference_frame == "world" || req.reference_frame == "map" || req.reference_frame == "/map")
       {
         ROS_INFO("ApplyBodyWrench: reference_frame is empty/world/map, using inertial frame, transferring from body relative to inertial frame");
         // FIXME: transfer to inertial frame
         gazebo::Pose3d target_to_reference = body->GetWorldPose();
         target_force = reference_force;
         target_torque = reference_torque;
 
       }
       else
       {
         ROS_ERROR("ApplyBodyWrench: reference_frame is not a valid link name");
         res.success = false;
         res.status_message = "ApplyBodyWrench: reference_frame not found";
         return false;
       }
 
       // apply wrench
       // schedule a job to do below at appropriate times:
       // body->SetForce(force)
       // body->SetTorque(torque)
       GazeboROSNode::WrenchBodyJob* wej = new GazeboROSNode::WrenchBodyJob;
       wej->body = body;
       wej->force = target_force;
       wej->torque = target_torque;
       wej->start_time = req.start_time;
       if (wej->start_time < ros::Time(gazebo::Simulator::Instance()->GetSimTime().Double()))
         wej->start_time = ros::Time(gazebo::Simulator::Instance()->GetSimTime().Double());
       wej->duration = req.duration;
       this->lock_.lock();
       this->wrench_body_jobs.push_back(wej);
       this->lock_.unlock();
 
       res.success = true;
       res.status_message = "";
       return true;
     }
 
 
   private:
     ros::NodeHandle rosnode_;
     ros::CallbackQueue gazebo_queue_;
     boost::thread* gazebo_callback_queue_thread_;
 
     ros::ServiceServer spawn_urdf_model_service_;
     ros::ServiceServer spawn_urdf_gazebo_service_;
     ros::ServiceServer delete_model_service_;
     ros::ServiceServer get_model_state_service_;
     ros::ServiceServer get_model_properties_service_;
     ros::ServiceServer get_world_properties_service_;
     ros::ServiceServer get_joint_properties_service_;
     ros::ServiceServer get_link_properties_service_;
     ros::ServiceServer get_link_state_service_;
     ros::ServiceServer set_link_properties_service_;
     ros::ServiceServer set_physics_properties_service_;
     ros::ServiceServer get_physics_properties_service_;
     ros::ServiceServer apply_body_wrench_service_;
     ros::ServiceServer set_joint_properties_service_;
     ros::ServiceServer set_model_state_service_;
     ros::ServiceServer apply_joint_effort_service_;
     ros::ServiceServer set_model_configuration_service_;
     ros::ServiceServer set_link_state_service_;
     ros::ServiceServer reset_simulation_service_;
     ros::ServiceServer reset_world_service_;
     ros::ServiceServer pause_physics_service_;
     ros::ServiceServer unpause_physics_service_;
     ros::ServiceServer clear_joint_forces_service_;
     ros::ServiceServer clear_body_wrenches_service_;
     ros::Subscriber    set_link_state_topic_;
     ros::Subscriber    set_model_state_topic_;
     ros::Publisher     pub_clock_;
     ros::Publisher     pub_link_states_;
     ros::Publisher     pub_model_states_;
     ros::Publisher     pub_gazebo_paused_;
     int                pub_link_states_connection_count_;
     int                pub_model_states_connection_count_;
 
     // physics dynamic reconfigure
     boost::thread* physics_reconfigure_thread_;
     bool physics_reconfigure_initialized_;
 
     ros::ServiceClient physics_reconfigure_set_client_;
     ros::ServiceClient physics_reconfigure_get_client_;
 
     void PhysicsReconfigureCallback(gazebo::PhysicsConfig &config, uint32_t level)
     {
       if (!physics_reconfigure_initialized_)
       {
         gazebo_msgs::GetPhysicsProperties srv;
         this->physics_reconfigure_get_client_.call(srv);
 
         config.time_step                   = srv.response.time_step;
         config.max_update_rate             = srv.response.max_update_rate;
         config.gravity_x                   = srv.response.gravity.x;
         config.gravity_y                   = srv.response.gravity.y;
         config.gravity_z                   = srv.response.gravity.z;
         config.auto_disable_bodies         = srv.response.ode_config.auto_disable_bodies;
         config.sor_pgs_precon_iters        = srv.response.ode_config.sor_pgs_precon_iters;
         config.sor_pgs_iters               = srv.response.ode_config.sor_pgs_iters;
         config.sor_pgs_rms_error_tol       = srv.response.ode_config.sor_pgs_rms_error_tol;
         config.sor_pgs_w                   = srv.response.ode_config.sor_pgs_w;
         config.contact_surface_layer       = srv.response.ode_config.contact_surface_layer;
         config.contact_max_correcting_vel  = srv.response.ode_config.contact_max_correcting_vel;
         config.cfm                         = srv.response.ode_config.cfm;
         config.erp                         = srv.response.ode_config.erp;
         config.max_contacts                = srv.response.ode_config.max_contacts;
         physics_reconfigure_initialized_ = true;
       }
       else
       {
         bool changed = false;
         gazebo_msgs::GetPhysicsProperties srv;
         this->physics_reconfigure_get_client_.call(srv);
 
         // check for changes
         if (config.time_step                      != srv.response.time_step)                                 changed = true;
         if (config.max_update_rate                != srv.response.max_update_rate)                           changed = true;
         if (config.gravity_x                      != srv.response.gravity.x)                                 changed = true;
         if (config.gravity_y                      != srv.response.gravity.y)                                 changed = true;
         if (config.gravity_z                      != srv.response.gravity.z)                                 changed = true;
         if (config.auto_disable_bodies            != srv.response.ode_config.auto_disable_bodies)            changed = true;
         if ((uint32_t)config.sor_pgs_precon_iters != srv.response.ode_config.sor_pgs_precon_iters)           changed = true;
         if ((uint32_t)config.sor_pgs_iters        != srv.response.ode_config.sor_pgs_iters)                  changed = true;
         if (config.sor_pgs_rms_error_tol          != srv.response.ode_config.sor_pgs_rms_error_tol)          changed = true;
         if (config.sor_pgs_w                      != srv.response.ode_config.sor_pgs_w)                      changed = true;
         if (config.contact_surface_layer          != srv.response.ode_config.contact_surface_layer)          changed = true;
         if (config.contact_max_correcting_vel     != srv.response.ode_config.contact_max_correcting_vel)     changed = true;
         if (config.cfm                            != srv.response.ode_config.cfm)                            changed = true;
         if (config.erp                            != srv.response.ode_config.erp)                            changed = true;
         if ((uint32_t)config.max_contacts         != srv.response.ode_config.max_contacts)                   changed = true;
 
         if (changed)
         {
           // pause simulation if requested
           gazebo_msgs::SetPhysicsProperties srv;
           srv.request.time_step                             = config.time_step                   ;
           srv.request.max_update_rate                       = config.max_update_rate             ;
           srv.request.gravity.x                             = config.gravity_x                   ;
           srv.request.gravity.y                             = config.gravity_y                   ;
           srv.request.gravity.z                             = config.gravity_z                   ;
           srv.request.ode_config.auto_disable_bodies        = config.auto_disable_bodies         ;
           srv.request.ode_config.sor_pgs_precon_iters       = config.sor_pgs_precon_iters        ;
           srv.request.ode_config.sor_pgs_iters              = config.sor_pgs_iters               ;
           srv.request.ode_config.sor_pgs_rms_error_tol      = config.sor_pgs_rms_error_tol       ;
           srv.request.ode_config.sor_pgs_w                  = config.sor_pgs_w                   ;
           srv.request.ode_config.contact_surface_layer      = config.contact_surface_layer       ;
           srv.request.ode_config.contact_max_correcting_vel = config.contact_max_correcting_vel  ;
           srv.request.ode_config.cfm                        = config.cfm                         ;
           srv.request.ode_config.erp                        = config.erp                         ;
           srv.request.ode_config.max_contacts               = config.max_contacts                ;
           this->physics_reconfigure_set_client_.call(srv);
           ROS_INFO("physics dynamics reconfigure update complete");
         }
         ROS_INFO("physics dynamics reconfigure complete");
       }
     }
 
     void PhysicsReconfigureNode()
     {
       ros::NodeHandle node_handle;
       this->physics_reconfigure_set_client_ = node_handle.serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");
       this->physics_reconfigure_get_client_ = node_handle.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
       this->physics_reconfigure_set_client_.waitForExistence();
       this->physics_reconfigure_get_client_.waitForExistence();
 
       // for dynamic reconfigure physics
       // for dynamic_reconfigure
       dynamic_reconfigure::Server<gazebo::PhysicsConfig> physics_reconfigure_srv;
       dynamic_reconfigure::Server<gazebo::PhysicsConfig>::CallbackType physics_reconfigure_f;
 
       physics_reconfigure_f = boost::bind(&GazeboROSNode::PhysicsReconfigureCallback, this, _1, _2);
       physics_reconfigure_srv.setCallback(physics_reconfigure_f);
 
       ROS_INFO("Starting to spin physics dynamic reconfigure node...");
       ros::Rate r(10);
       while(ros::ok())
       {
         ros::spinOnce();
         r.sleep();
       }
     }
 
 
     tf::TransformBroadcaster tfbr;
 
     boost::mutex lock_;
 
     std::string xmlPrefix_;
     std::string xmlSuffix_;
 
     void SetupTransform(btTransform &transform, urdf::Pose pose)
     {
         btMatrix3x3 mat;
         mat.setRotation(btQuaternion(pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w));
         transform = btTransform(mat,btVector3(pose.position.x,pose.position.y,pose.position.z));
     }
     void SetupTransform(btTransform &transform, geometry_msgs::Pose pose)
     {
         btMatrix3x3 mat;
         mat.setRotation(btQuaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
         transform = btTransform(mat,btVector3(pose.position.x,pose.position.y,pose.position.z));
     }
 
 
 
 
     bool IsURDF(std::string model_xml)
     {
       TiXmlDocument doc_in;
       doc_in.Parse(model_xml.c_str());
       if (doc_in.FirstChild("robot"))
         return true;
       else
         return false;
     }
     bool IsColladaData(const std::string& data) // copied from urdf/src/model.cpp
     {
       return data.find("<COLLADA") != std::string::npos;
     }
     bool IsGazeboModelXML(std::string model_xml)
     {
       TiXmlDocument doc_in;
       doc_in.Parse(model_xml.c_str());
       if (doc_in.FirstChild("model:physical"))
         return true;
       else
         return false;
     }
 
     gazebo::Joint* getJointByName(std::string joint_name)
     {
       // look through all models in the world, search for body name that matches frameName
       std::vector<gazebo::Model*> all_models = gazebo::World::Instance()->GetModels();
       for (std::vector<gazebo::Model*>::iterator miter = all_models.begin(); miter != all_models.end(); miter++)
       {
         if ((*miter) && (*miter)->GetJoint(joint_name))
           return (*miter)->GetJoint(joint_name);
       }
       return NULL;
     }
 
 
     class WrenchBodyJob
     {
       public:
         gazebo::Body* body;
         gazebo::Vector3 force;
         gazebo::Vector3 torque;
         ros::Time start_time;
         ros::Duration duration;
     };
 
     class ForceJointJob
     {
       public:
         gazebo::Joint* joint;
         double force; // should this be a array?
         ros::Time start_time;
         ros::Duration duration;
     };
 
     std::vector<GazeboROSNode::WrenchBodyJob*> wrench_body_jobs;
     std::vector<GazeboROSNode::ForceJointJob*> force_joint_jobs;
 
     double lastWrenchBodyUpdateTime;
     double lastForceJointUpdateTime;
 
     void wrenchBodySchedulerSlot()
     {
       if (this->lastWrenchBodyUpdateTime == gazebo::Simulator::Instance()->GetSimTime().Double())
         return;
       // MDMutex locks in case model is getting deleted, don't have to do this if we delete jobs first
       // boost::recursive_mutex::scoped_lock lock(*gazebo::Simulator::Instance()->GetMDMutex());
       this->lock_.lock();
       for (std::vector<GazeboROSNode::WrenchBodyJob*>::iterator iter=this->wrench_body_jobs.begin();iter!=this->wrench_body_jobs.end();)
       {
         // check times and apply wrench if necessary
         if (ros::Time(gazebo::Simulator::Instance()->GetSimTime().Double()) >= (*iter)->start_time)
           if (ros::Time(gazebo::Simulator::Instance()->GetSimTime().Double()) <= (*iter)->start_time+(*iter)->duration ||
               (*iter)->duration.toSec() < 0.0)
           {
             if ((*iter)->body) // if body exists
             {
               (*iter)->body->SetForce((*iter)->force);
               (*iter)->body->SetTorque((*iter)->torque);
             }
             else
               (*iter)->duration.fromSec(0.0); // mark for delete
           }
 
         if (ros::Time(gazebo::Simulator::Instance()->GetSimTime().Double()) > (*iter)->start_time+(*iter)->duration &&
             (*iter)->duration.toSec() >= 0.0)
         {
           // remove from queue once expires
           delete (*iter);
           this->wrench_body_jobs.erase(iter);
         }
         else
           iter++;
       }
       this->lastWrenchBodyUpdateTime = gazebo::Simulator::Instance()->GetSimTime().Double();
       this->lock_.unlock();
     }
 
     void forceJointSchedulerSlot()
     {
       if (this->lastForceJointUpdateTime == gazebo::Simulator::Instance()->GetSimTime().Double())
         return;
       // MDMutex locks in case model is getting deleted, don't have to do this if we delete jobs first
       // boost::recursive_mutex::scoped_lock lock(*gazebo::Simulator::Instance()->GetMDMutex());
       this->lock_.lock();
       for (std::vector<GazeboROSNode::ForceJointJob*>::iterator iter=this->force_joint_jobs.begin();iter!=this->force_joint_jobs.end();)
       {
         // check times and apply force if necessary
         if (ros::Time(gazebo::Simulator::Instance()->GetSimTime().Double()) >= (*iter)->start_time)
           if (ros::Time(gazebo::Simulator::Instance()->GetSimTime().Double()) <= (*iter)->start_time+(*iter)->duration ||
               (*iter)->duration.toSec() < 0.0)
             {
               if ((*iter)->joint) // if joint exists
                 (*iter)->joint->SetForce(0,(*iter)->force);
               else
                 (*iter)->duration.fromSec(0.0); // mark for delete
             }
 
         if (ros::Time(gazebo::Simulator::Instance()->GetSimTime().Double()) > (*iter)->start_time+(*iter)->duration &&
             (*iter)->duration.toSec() >= 0.0)
         {
           // remove from queue once expires
           this->force_joint_jobs.erase(iter);
         }
         else
           iter++;
       }
       this->lastForceJointUpdateTime = gazebo::Simulator::Instance()->GetSimTime().Double();
       this->lock_.unlock();
     }
 
     void publishSimTime()
     {
       gazebo::Time currentTime = gazebo::Simulator::Instance()->GetSimTime();
       rosgraph_msgs::Clock ros_time_;
       ros_time_.clock.fromSec(currentTime.Double());
       /***************************************************************/
       /*                                                             */
       /*  publish time to ros                                        */
       /*                                                             */
       /***************************************************************/
       this->pub_clock_.publish(ros_time_);
     }
 
     void publishLinkStates()
     {
       gazebo_msgs::LinkStates link_states;
       // fill link_states
       const std::vector<gazebo::Model*> models = gazebo::World::Instance()->GetModels();
       for (std::vector<gazebo::Model*>::const_iterator miter = models.begin(); miter != models.end(); miter++)
       {
         const std::vector< gazebo::Entity* > entities = (*miter)->GetChildren();
         for (std::vector< gazebo::Entity* >::const_iterator eiter = entities.begin();eiter!=entities.end();eiter++)
         {
           gazebo::Body* body = dynamic_cast<gazebo::Body*>(*eiter);
           if (body)
           {
             link_states.name.push_back(body->GetCompleteScopedName());
             geometry_msgs::Pose pose;
             gazebo::Pose3d  body_pose = body->GetWorldPose(); // - this->myBody->GetCoMPose();
             gazebo::Vector3 pos = body_pose.pos;
             gazebo::Quatern rot = body_pose.rot;
             pose.position.x = pos.x;
             pose.position.y = pos.y;
             pose.position.z = pos.z;
             pose.orientation.w = rot.u;
             pose.orientation.x = rot.x;
             pose.orientation.y = rot.y;
             pose.orientation.z = rot.z;
             link_states.pose.push_back(pose);
             gazebo::Vector3 linear_vel  = body->GetWorldLinearVel();
             gazebo::Vector3 angular_vel = body->GetWorldAngularVel();
             geometry_msgs::Twist twist;
             twist.linear.x = linear_vel.x;
             twist.linear.y = linear_vel.y;
             twist.linear.z = linear_vel.z;
             twist.angular.x = angular_vel.x;
             twist.angular.y = angular_vel.y;
             twist.angular.z = angular_vel.z;
             link_states.twist.push_back(twist);
           }
         }
       }
       this->pub_link_states_.publish(link_states);
     }
 
     void publishModelStates()
     {
       gazebo_msgs::ModelStates model_states;
       // fill model_states
       const std::vector<gazebo::Model*> models = gazebo::World::Instance()->GetModels();
       for (std::vector<gazebo::Model*>::const_iterator miter = models.begin(); miter != models.end(); miter++)
       {
         model_states.name.push_back((*miter)->GetCompleteScopedName());
         geometry_msgs::Pose pose;
         gazebo::Pose3d  model_pose = (*miter)->GetWorldPose(); // - this->myBody->GetCoMPose();
         gazebo::Vector3 pos = model_pose.pos;
         gazebo::Quatern rot = model_pose.rot;
         pose.position.x = pos.x;
         pose.position.y = pos.y;
         pose.position.z = pos.z;
         pose.orientation.w = rot.u;
         pose.orientation.x = rot.x;
         pose.orientation.y = rot.y;
         pose.orientation.z = rot.z;
         model_states.pose.push_back(pose);
         gazebo::Vector3 linear_vel  = (*miter)->GetWorldLinearVel();
         gazebo::Vector3 angular_vel = (*miter)->GetWorldAngularVel();
         geometry_msgs::Twist twist;
         twist.linear.x = linear_vel.x;
         twist.linear.y = linear_vel.y;
         twist.linear.z = linear_vel.z;
         twist.angular.x = angular_vel.x;
         twist.angular.y = angular_vel.y;
         twist.angular.z = angular_vel.z;
         model_states.twist.push_back(twist);
       }
       this->pub_model_states_.publish(model_states);
     }
 
 };
 
 // Main function
 int main(int argc, char **argv)
 {
 
   // start a gazebo ros node
   ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler);//|ros::init_options::AnonymousName);
 
   // force a cpu affinity for CPU 0, this slow down sim by about 4X
   // cpu_set_t cpuSet;
   // CPU_ZERO(&cpuSet);
   // CPU_SET(0, &cpuSet);
   // sched_setaffinity( 0, sizeof(cpuSet), &cpuSet);
 
   //Application Setup
   if (ParseArgs(argc, argv) != 0)
   {
     ros::shutdown();
     return -1;
   }
 
   PrintVersion();
 
   if (signal(SIGINT, SignalHandler) == SIG_ERR)
   {
     std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
     ros::shutdown();
     return -1;
   }
 
   gazebo::Simulator::Instance()->SetCreateOgreLog( optOgreLog );
   gazebo::Simulator::Instance()->SetGuiEnabled( optGuiEnabled );
   gazebo::Simulator::Instance()->SetRenderEngineEnabled( optRenderEngineEnabled );
 
   // setting Gazebo Path/Resources Configurations
   //   GAZEBO_RESOURCE_PATH, GAZEBO_PLUGIN_PATH and OGRE_RESOURCE_PATH for
   //   GazeboConfig::gazeboPaths, GazeboConfig::pluginPaths and GazeboConfig::ogrePaths
   // by adding paths to GazeboConfig based on ros::package
   // optional: setting environment variables according to ROS
   //           e.g. setenv("OGRE_RESOURCE_PATH",ogre_package_path.c_str(),1);
   // set ogre library paths by searching for package named ogre
   gazebo::Simulator::Instance()->GetGazeboConfig()->ClearOgrePaths();
   std::string ogre_package_name("ogre"); // @todo: un hardcode this???
   std::string ogre_package_path = ros::package::getPath(ogre_package_name)+std::string("/ogre/lib/OGRE");
   if (ogre_package_path.empty())
     ROS_DEBUG("Package[%s] does not exist, assuming user must have set OGRE_RESOURCE_PATH already.",ogre_package_name.c_str());
   else
   {
     ROS_DEBUG("ogre path %s",ogre_package_path.c_str());
     gazebo::Simulator::Instance()->GetGazeboConfig()->AddOgrePaths(ogre_package_path);
   }
 
   // set gazebo media paths by adding all packages that exports "gazebo_media_path" for gazebo
   gazebo::Simulator::Instance()->GetGazeboConfig()->ClearGazeboPaths();
   std::vector<std::string> gazebo_media_paths;
   ros::package::getPlugins("gazebo","gazebo_media_path",gazebo_media_paths);
   for (std::vector<std::string>::iterator iter=gazebo_media_paths.begin(); iter != gazebo_media_paths.end(); iter++)
   {
     ROS_DEBUG("med path %s",iter->c_str());
     gazebo::Simulator::Instance()->GetGazeboConfig()->AddGazeboPaths(iter->c_str());
   }
 
   // set gazebo plugins paths by adding all packages that exports "plugin" for gazebo
   gazebo::Simulator::Instance()->GetGazeboConfig()->ClearPluginPaths();
   std::vector<std::string> plugin_paths;
   ros::package::getPlugins("gazebo","plugin_path",plugin_paths);
   for (std::vector<std::string>::iterator iter=plugin_paths.begin(); iter != plugin_paths.end(); iter++)
   {
     ROS_DEBUG("plugin path %s",(*iter).c_str());
     gazebo::Simulator::Instance()->GetGazeboConfig()->AddPluginPaths(iter->c_str());
   }
 
   // set .gazeborc path to something else, so we don't pick up default ~/.gazeborc
   std::string gazeborc = ros::package::getPath("gazebo")+"/.do_not_use_gazeborc";
   setenv("GAZEBORC",gazeborc.c_str(),1);
 
   //Load the simulator
   try
   {
     if (optWorldParam)
       gazebo::Simulator::Instance()->LoadWorldString(worldParamData, optServerId);
     else
       gazebo::Simulator::Instance()->LoadWorldFile(worldFileName, optServerId);
     
     gazebo::Simulator::Instance()->SetTimeout(optTimeout);
     gazebo::Simulator::Instance()->SetPhysicsEnabled(optPhysicsEnabled);
   }
   catch (gazebo::GazeboError e)
   {
     std::cerr << "Error Loading Gazebo" << std::endl;
     std::cerr << e << std::endl;
     gazebo::Simulator::Instance()->Fini();
     ros::shutdown();
     return -1;
   }
 
   // Initialize the simulator
   try
   {
     gazebo::Simulator::Instance()->Init();
     gazebo::Simulator::Instance()->SetPaused(optPaused);
   }
   catch (gazebo::GazeboError e)
   {
     std::cerr << "Initialization failed" << std::endl;
     std::cerr << e << std::endl;
     gazebo::Simulator::Instance()->Fini();
     ros::shutdown();
     return -1;
   }
 
   // start a node
   GazeboROSNode grn;
 
   // Main loop of the simulator
   try
   {
     gazebo::Simulator::Instance()->MainLoop();
   }
   catch (gazebo::GazeboError e)
   {
     std::cerr << "Main simulation loop failed" << std::endl;
     std::cerr << e << std::endl;
     gazebo::Simulator::Instance()->Fini();
     ros::shutdown();
     return -1;
   }
 
   // Finalization and clean up
   try
   {
     gazebo::Simulator::Instance()->Fini();
   }
   catch (gazebo::GazeboError e)
   {
     std::cerr << "Finalization failed" << std::endl;
     std::cerr << e << std::endl;
     ros::shutdown();
     return -1;
   }
 
   printf("Done.\n");
   ros::shutdown();
   return 0;
 }