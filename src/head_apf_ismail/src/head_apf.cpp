/*
 * Author       : Batuhan Vatan
 * Date         : 26 Aug '23
 * Description  : POSITION CONTROLLER DIDN'T IMPLEMENTED! For both simulation and real life use VELOCITY CONTROLLER.

  TODO: Modify this and HumanBB callback for multi-person. Currently, only one person is assumed to be present and to be tracked.
*/

#include "head_apf/head_apf.hpp"

// HeadAPF Class implementations
HeadAPF::HeadAPF(float loopRate, std::string target_topic_name) : nh_("~"),
                                                                  loopRate_(loopRate),
                                                                  pan_prev_time_(ros::Time::now().toSec()),
                                                                  tilt_prev_time_(ros::Time::now().toSec()),
                                                                  newPanVelReceived_(false),
                                                                  newTiltVelReceived_(false)
{
  // Target Subscriber
  targetSub_ = nh_.subscribe(target_topic_name, 1, &HeadAPF::targetCallback, this);

  // PTU subscribers and publishers
  panVelSub_ = nh_.subscribe(panVelTopicSub, 1, &HeadAPF::panVelCallback, this);
  tiltVelSub_ = nh_.subscribe(tiltPosTopicSub, 1, &HeadAPF::tiltVelCallback, this);

#if CONTROLLER == POSITION_CONTROLLER
  panPosSub_ = nh_.subscribe(panPosTopicSub, 1, &HeadAPF::panPosCallback, this);
  tiltPosSub_ = nh_.subscribe(tiltPosTopicSub, 1, &HeadAPF::tiltPosCallback, this);
  panPosPub_ = nh_.advertise<std_msgs::Float64>(panPosTopicPub, 1);
  tiltPosPub_ = nh_.advertise<std_msgs::Float64>(tiltPosTopicPub, 1);
#elif CONTROLLER == VELOCITY_CONTROLLER
  jointStatesSub_ = nh_.subscribe("/joint_states", 1, &HeadAPF::jointStatesCallback, this); // GT for poses

  panPosFromVelSub_ = nh_.subscribe(panVelTopicSub, 1, &HeadAPF::panPosFromVelCallback, this);
  tiltPosFromVelSub_ = nh_.subscribe(tiltVelTopicSub, 1, &HeadAPF::tiltPosFromVelCallback, this);

  panVelPub_ = nh_.advertise<std_msgs::Float64>(panVelTopicPub, 1);
  tiltVelPub_ = nh_.advertise<std_msgs::Float64>(tiltVelTopicPub, 1);
#endif
}

HeadAPF::~HeadAPF()
{
  // TODO: Publish zero velocity to PTU (For real system)

  // PTU subscribers and publishers
  panVelSub_.shutdown();
  tiltVelSub_.shutdown();
}

// Public functions - Start

void HeadAPF::apfStep()
{
  
  if (VERBOSE)
  {
    ROS_WARN("APF STEP STARTS");
  }
  apfPublish();
  loopRate_.sleep();
  if (VERBOSE)
    ROS_WARN("APF STEP ENDS");
}

void HeadAPF::targetCallback(const geometry_msgs::Point::ConstPtr &msg)
{
  panTargetAngle_ = msg->x;
  tiltTargetAngle_ = msg->y;
}

#if CONTROLLER == POSITION_CONTROLLER
void HeadAPF::setPanPos(float panPos)
{
  publishPanPos(panPos);
}
void HeadAPF::setTiltPos(float tiltPos)
{
  publishTiltPos(tiltPos);
}

#elif CONTROLLER == VELOCITY_CONTROLLER
void HeadAPF::setPanVel(float panVel)
{
  publishPanVel(panVel);
}

void HeadAPF::setTiltVel(float tiltVel)
{
  publishTiltVel(tiltVel);
}
#endif

// Public functions - End

// PTU - Start
#if CONTROLLER == POSITION_CONTROLLER
void HeadAPF::panPosCallback(const control_msgs::JointControllerState::ConstPtr &msg)
{
  rad2deg(msg->process_value, &panAngleEncoder_);
  if (VERBOSE)
  {
    ROS_INFO("panPosCallback: Pan angle: %f", panAngleEncoder_);
  }
}

void HeadAPF::tiltPosCallback(const control_msgs::JointControllerState::ConstPtr &msg)
{
  rad2deg(msg->process_value, &tiltAngleEncoder_);
  if (VERBOSE)
  {
    ROS_INFO("tiltPosCallback: Tilt angle: %f", tiltAngleEncoder_);
  }
}

void HeadAPF::panVelCallback(const control_msgs::JointControllerState::ConstPtr &msg)
{
  radpersec2RPM(msg->process_value_dot, &panVelCurrent_);
  if (VERBOSE)
  {
    ROS_INFO("panVelCallback: Pan velocity: %f", panVelCurrent_);
  }
}
void HeadAPF::tiltVelCallback(const control_msgs::JointControllerState::ConstPtr &msg)
{
  radpersec2RPM(msg->process_value_dot, &tiltVelCurrent_);
  if (VERBOSE)
  {
    ROS_INFO("tiltVelCallback: Tilt velocity: %f", tiltVelCurrent_);
  }
}

#elif CONTROLLER == VELOCITY_CONTROLLER
void HeadAPF::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &joint_states)
{
  for (int i = 0; i < joint_states->name.size(); i++)
  {
    if (joint_states->name[i] == "pan_joint")
    {
      rad2deg(joint_states->position[i], &panAngleEncoder_);
    }
    else if (joint_states->name[i] == "tilt_joint")
    {
      rad2deg(joint_states->position[i], &tiltAngleEncoder_);
    }
  }
  if (VERBOSE)
  {
    ROS_INFO("jointStatesCallback: Pan angle: %f, Tilt angle: %f", panAngleEncoder_, tiltAngleEncoder_);
  }
}

void HeadAPF::panPosFromVelCallback(const control_msgs::JointControllerState::ConstPtr &msg)
{
  if (!newPanVelReceived_)
  {
    return;
  }
  panAngleCurrent_ += RPM2degpersec(panVelCurrent_) * pan_dt_; // deg
  if (VERBOSE)
  {
    ROS_INFO("tempFunc_Drift: Pan angle: %f", panAngleCurrent_);
  }
  newPanVelReceived_ = false;
}

void HeadAPF::tiltPosFromVelCallback(const control_msgs::JointControllerState::ConstPtr &msg)
{
  if (!newTiltVelReceived_)
  {
    return;
  }
  tiltAngleCurrent_ += RPM2degpersec(tiltVelCurrent_) * tilt_dt_; // deg
  if (VERBOSE)
  {
    ROS_INFO("tempFunc_Drift: Tilt angle: %f", tiltAngleCurrent_);
  }
  newTiltVelReceived_ = false;
}

void HeadAPF::panVelCallback(const control_msgs::JointControllerState::ConstPtr &msg)
{
  radpersec2RPM(msg->process_value, &panVelCurrent_);
  if (VERBOSE)
  {
    ROS_INFO("panVelCallback: Pan velocity: %f", panVelCurrent_);
  }

  newPanVelReceived_ = true;
  float current_time = ros::Time::now().toSec();
  pan_dt_ = current_time - pan_prev_time_;
  pan_prev_time_ = current_time;
}

void HeadAPF::tiltVelCallback(const control_msgs::JointControllerState::ConstPtr &msg)
{
  radpersec2RPM(msg->process_value, &tiltVelCurrent_);
  if (VERBOSE)
  {
    ROS_INFO("tiltVelCallback: Tilt velocity: %f", tiltVelCurrent_);
  }
  newTiltVelReceived_ = true;
  float current_time = ros::Time::now().toSec();
  tilt_dt_ = current_time - tilt_prev_time_;
  tilt_prev_time_ = current_time;
}
#endif

#if CONTROLLER == POSITION_CONTROLLER
void HeadAPF::publishPanPos(float panPos) // Always degree as an argument then ->  SIM: rad, REAL: degree
{
  std_msgs::Float64 panMsg;
  
  panMsg.data = panPos; // rad
  panPosPub_.publish(panMsg);
  if (VERBOSE)
  {
    ROS_INFO("publishPanPos: Pan angle: %f", panPos);
  }
}

void HeadAPF::publishTiltPos(float tiltPos) // Always degree as an argument then ->  SIM: rad, REAL: degree
{
  std_msgs::Float64 tiltMsg;
  tiltMsg.data = tiltPos; // rad
  tiltPosPub_.publish(tiltMsg);
  if (VERBOSE)
  {
    ROS_INFO("publishTiltPos: Tilt angle: %f", tiltPos);
  }
}

#elif CONTROLLER == VELOCITY_CONTROLLER
void HeadAPF::publishPanVel(float panVel) // Always RPM as an argument then ->  SIM: rad/s, REAL: RPM
{
  std_msgs::Float64 panMsg;
  panMsg.data = RPM2radpersec(panVel);
  panVelPub_.publish(panMsg);
  if (VERBOSE)
  {
    ROS_INFO("publishPanVel: Pan velocity: %f", panVel);
  }
}

void HeadAPF::publishTiltVel(float tiltVel) // Always RPM as an argument then -> SIM: rad/s, REAL: RPM
{
  std_msgs::Float64 tiltMsg;
  tiltMsg.data = RPM2radpersec(tiltVel);
  tiltVelPub_.publish(tiltMsg);
  if (VERBOSE)
  {
    ROS_INFO("publishTiltVel: Tilt velocity: %f", tiltVel);
  }
}

#endif

// PTU - End

// APF - Start

cv::Point2f HeadAPF::attractiveField(float x, float y)
{
  cv::Point2f phi;
  phi.x = (1 / kAttractive_) * pow(x - panTargetAngle_, 2);
  phi.y = (1 / kAttractive_) * pow(y - tiltTargetAngle_, 2);

  return phi;
}

cv::Point2f HeadAPF::calculateVelocity()
{
  std::vector<bool> reached = checkGoalReached();

  // Numerical Gradient Calculation
  cv::Point2f Dphi;
  Dphi.x = (attractiveField(deg2rad(panAngleEncoder_ + gradStep_), deg2rad(tiltAngleEncoder_)).x - attractiveField(deg2rad(panAngleEncoder_ - gradStep_), deg2rad(tiltAngleEncoder_)).x) / (2 * deg2rad(gradStep_));
  Dphi.y = (attractiveField(deg2rad(panAngleEncoder_), deg2rad(tiltAngleEncoder_ + gradStep_)).y - attractiveField(deg2rad(panAngleEncoder_), deg2rad(tiltAngleEncoder_ - gradStep_)).y) / (2 * deg2rad(gradStep_));

  // Normalization
  cv::Point2f DphiNorm;
  DphiNorm.x = Dphi.x / sqrt(pow(Dphi.x, 2) + pow(Dphi.y, 2));
  DphiNorm.y = Dphi.y / sqrt(pow(Dphi.x, 2) + pow(Dphi.y, 2));

  // // Not conventional normalization, based on max difference possible (FOV/2)
  // cv::Point2f DphiNorm;
  // DphiNorm.x = Dphi.x * (xScalingGrad_ / sqrt(pow(xScalingGrad_, 2) + pow(yScalingGrad_, 2)));
  // DphiNorm.y = Dphi.y * (yScalingGrad_ / sqrt(pow(xScalingGrad_, 2) + pow(yScalingGrad_, 2)));

  cv::Point2f velocity;
  if (!reached[0])
  {
    velocity.x = RPM2radpersec(maxPanVel_) * DphiNorm.x;
  }
  else
  {
    velocity.x = 0.0;
  }
  if (!reached[1])
  {
    velocity.y = RPM2radpersec(maxTiltVel_) * DphiNorm.y;
  }
  else
  {
    velocity.y = 0.0;
  }

  return velocity;
}

void HeadAPF::apfPublish()
{
  cv::Point2f velocity = calculateVelocity();
#if CONTROLLER == VELOCITY_CONTROLLER
  float panVel = velocity.x;   // rad/s
  float tiltVel = -velocity.y; // rad/s
  checkAngleLimits(&panVel, &tiltVel, velocity.x > 0, (-velocity.y) > 0);
  publishPanVel(panVel);
  publishTiltVel(tiltVel);

  if (VERBOSE)
  {
    ROS_INFO("apfPublish: Pan velocity: %f, Tilt velocity: %f", velocity.x, velocity.y);
    ROS_INFO("apfPublish Published velocities: Pan: %f, Tilt: %f", panVel, tiltVel);
  }
#endif
}
// APF - End

// Utils - Start

std::vector<bool> HeadAPF::checkGoalReached()
{
  bool temp;
  std::vector<bool> goalReached;
  temp = (abs(deg2rad(panAngleEncoder_ - panTargetAngle_)) < deg2rad(reachGoalThresholdPan_));
  goalReached.push_back(temp);
  temp = (abs(deg2rad(tiltAngleEncoder_ - tiltTargetAngle_)) < deg2rad(reachGoalThresholdTilt_));
  goalReached.push_back(temp);
  return goalReached;
}

#if CONTROLLER == VELOCITY_CONTROLLER
void HeadAPF::checkAngleLimits(float *panVel, float *tiltVel, bool panDirection, bool tiltDirection)
{
  if ((panAngleEncoder_ > PAN_MAX_DEG) && panDirection)
  {
    *panVel = 0.0;
  }
  else if ((panAngleEncoder_ < PAN_MIN_DEG) && !panDirection)
  {
    *panVel = 0.0;
  }

  if ((tiltAngleEncoder_ > TILT_MAX_DEG) && panDirection)
  {
    *tiltVel = 0.0;
  }
  else if ((tiltAngleEncoder_ < TILT_MIN_DEG) && !panDirection)
  {
    *tiltVel = 0.0;
  }
}
#endif

void HeadAPF::printError()
{
  ROS_WARN("Pan current: %f, Tilt current: %f", panAngleCurrent_, tiltAngleCurrent_);
  ROS_WARN("Pan encoder: %f, Tilt encoder: %f", panAngleEncoder_, tiltAngleEncoder_);
  ROS_WARN("Pan error: %f, Tilt error: %f", panAngleEncoder_ - panAngleCurrent_, tiltAngleEncoder_ - tiltAngleCurrent_);
}
// Utils - End