//This code is for the part d

// Libraries
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Dense>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hokuyo_go");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());


  // Creating necessary objects
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("body");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // Print the current values of joints
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }
  
  //set the values
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  
  // getting transform matrix of end effector and link-3
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("end_effector");
  //const Eigen::Isometry3d& link3_state = kinematic_state->getGlobalLinkTransform("");
  
  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation matrix of end effector case 1: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation matrix of end effector case 1: \n" << end_effector_state.rotation() << "\n");
  
  // printing point of link-3
  Eigen::Vector3d point_link3(0, 0, 0);
  //ROS_INFO_STREAM("Link-3 workspace point case 1: \n" << link3_state.rotation()*point_link3+link3_state.translation() << "\n");
  //geometry_msgs::Pose target_pose1;
  //tf::poseEigenToMsg (end_effector_state , target_pose1);
  // setting the second values
  joint_values[0] = 1.57;
  joint_values[1] = -0.4;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
  
  // printing current state into terminal
  for (std::size_t j = 0; j < joint_names.size(); ++j)
  {
    ROS_INFO("Joint %s: %f", joint_names[j].c_str(), joint_values[j]);
  }
  
  // getting transform matrix of end effector
  const Eigen::Isometry3d& end_effector_state_2 = kinematic_state->getGlobalLinkTransform("end_effector");
  
  // printing results
  ROS_INFO_STREAM("Translation case 2: \n" << end_effector_state_2.translation() << "\n");
  ROS_INFO_STREAM("Rotation case 2: \n" << end_effector_state_2.rotation() << "\n");
  //ROS_INFO_STREAM("Link-3 workspace point case 2: \n" << link3_state.rotation()*point_link3+link3_state.translation() << "\n");
  
  // END
  ros::shutdown();
  return 0;
}
