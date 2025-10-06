/*
 * Author       : Batuhan Vatan
 * Date         : 26 Aug '23
 * Description  : POSITION CONTROLLER DIDN'T IMPLEMENTED! For both simulation and real life use VELOCITY CONTROLLER.

  TODO: Modify this and HumanBB callback for multi-person. Currently, only one person is assumed to be present and to be tracked.
*/

#ifndef _HEAD_APF_HPP_
#define _HEAD_APF_HPP_
#pragma once

#include "head_apf/utility.hpp"

class HeadAPF
{
public:
  HeadAPF(float loopRate, std::string target_topic_name);
  ~HeadAPF();

  void apfStep();
#if CONTROLLER == POSITION_CONTROLLER
  void setPanPos(float panPos);   // DEGREE
  void setTiltPos(float tiltPos); // DEGREE
#elif CONTROLLER == VELOCITY_CONTROLLER
  void setPanVel(float panVel);   // RPM
  void setTiltVel(float tiltVel); // RPM
#endif

  void printError();

private:
  ros::NodeHandle nh_;
  ros::Rate loopRate_;

  ros::Subscriber targetSub_;
  void targetCallback(const geometry_msgs::Point::ConstPtr &msg);

  float panTargetAngle_{0.0}, tiltTargetAngle_{0.0};

  // Human detector

  // PTU
  ros::Subscriber panPosSub_;
  ros::Subscriber tiltPosSub_;
  ros::Subscriber panVelSub_;
  ros::Subscriber tiltVelSub_;

  ros::Publisher panPosPub_;
  ros::Publisher tiltPosPub_;
  ros::Publisher panVelPub_;
  ros::Publisher tiltVelPub_;

  float pan_prev_time_{0.0};
  float pan_dt_{0.0};

  float tilt_prev_time_{0.0};
  float tilt_dt_{0.0};

#if CONTROLLER == POSITION_CONTROLLER
  void panPosCallback(const control_msgs::JointControllerState::ConstPtr &msg);
  void tiltPosCallback(const control_msgs::JointControllerState::ConstPtr &msg);
#elif CONTROLLER == VELOCITY_CONTROLLER
  ros::Subscriber jointStatesSub_;
  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_states);
  ros::Subscriber panPosFromVelSub_;
  ros::Subscriber tiltPosFromVelSub_;
  bool newPanVelReceived_;
  bool newTiltVelReceived_;
  void panPosFromVelCallback(const control_msgs::JointControllerState::ConstPtr &msg);
  void tiltPosFromVelCallback(const control_msgs::JointControllerState::ConstPtr &msg);
#endif

  void panVelCallback(const control_msgs::JointControllerState::ConstPtr &msg);
  void tiltVelCallback(const control_msgs::JointControllerState::ConstPtr &msg);

#if CONTROLLER == POSITION_CONTROLLER
  void
  publishPanPos(float panPos);        // SIM: RAD, REAL-LIFE: DEGREE
  void publishTiltPos(float tiltPos); // SIM: RAD, REAL-LIFE: DEGREE
#elif CONTROLLER == VELOCITY_CONTROLLER
  void publishPanVel(float panVel);   // SIM: RAD/S, REAL-LIFE: RPM
  void publishTiltVel(float tiltVel); // SIM: RAD/S, REAL-LIFE: RPM
#endif
  // PTU

  // APF
  static constexpr float panOffset_ = 0.0; //buraya bak
  static constexpr float tiltOffset_ = 0.0;

  float panAngleCurrent_{0.0}, tiltAngleCurrent_{0.0}; // rad
  float panVelCurrent_{0.0}, tiltVelCurrent_{0.0};     // rad/s
  float panAngleEncoder_{0.0}, tiltAngleEncoder_{0.0}; // rad, Encoder names are misleading, they are obtained from simulation directly. Naming is for consistency.

  // APF Parameters
  static constexpr float kAttractive_ = 1;              // For attractive field tuning, based on function we want to create we can play with this.
  static constexpr float reachGoalThresholdPan_ = 1.0;  // In degree
  static constexpr float reachGoalThresholdTilt_ = 1.0; // In degree
  static constexpr float gradStep_ = 1.0;               // for gradient calculation, in degrees
  static constexpr float maxPanVel_ = 50.00;            // RPM
  static constexpr float maxTiltVel_ = 50.00;           // RPM

  cv::Point2f attractiveField(float x, float y);
  cv::Point2f calculateVelocity();
  void apfPublish();
  // APF

  // Utility"
  std::vector<bool> checkGoalReached();
  void checkAngleLimits(float *panVel, float *tiltVel, bool panDirection, bool tiltDirection);
};

#endif // HEAD_APF_HPP_