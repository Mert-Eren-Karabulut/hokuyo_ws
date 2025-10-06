/*
 * Author       : Batuhan Vatan
 * Date         : 26 Aug '23
 * Description  : Definitions are based on ZED2 Camera, Dynamixel MX-64T motors and Arduino Mega 2560 with Dynamixel Shield 1.0.
 */

#ifndef _DEFINITIONS_HPP_
#define _DEFINITIONS_HPP_
#pragma once

#define PI 3.14159265358979
// PTU controllers for SIMULATION;
#define POSITION_CONTROLLER 1
#define VELOCITY_CONTROLLER 2

// Turn simulation on or off
#define VERBOSE false
#define CONTROLLER VELOCITY_CONTROLLER

// Pan and tilt msgs
#include "control_msgs/JointControllerState.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"

// Topic names for simulation
#define panPosTopicSub "/pan_controller/state"   // process_value.data
#define tiltPosTopicSub "/tilt_controller/state" // process_value.data
#define panVelTopicSub "/pan_controller/state"   // process_value_dot.data
#define tiltVelTopicSub "/pan_controller/state"  // process_value_dot.data

// Change based on what controllers you use!
#if CONTROLLER == POSITION_CONTROLLER
#define panPosTopicPub "/pan_controller/command"
#define tiltPosTopicPub "/tilt_controller/command"
#elif CONTROLLER == VELOCITY_CONTROLLER
#define panVelTopicPub "/pan_controller/command"
#define tiltVelTopicPub "/tilt_controller/command"
#endif

// Motor parameters
#define MOTOR_MIN_PULSE 0
#define MOTOR_MAX_PULSE 4095
#define MOTOR_MIN_ANGLE 0.0
#define MOTOR_MAX_ANGLE 360.0

// Pan and tilt motor limits
#define PAN_MIN_DEG -180.000421 // -90-0
#define PAN_MAX_DEG 180.000421  // 90-0
#define PAN_MIN_PULSE 0
#define PAN_MAX_PULSE 4095

#define TILT_MIN_DEG -50.002027// 140.01-180
#define TILT_MAX_DEG 50.002027 // 219.99-180
#define TILT_MIN_PULSE 1593
#define TILT_MAX_PULSE 2503

#endif // DEFINITIONS_HPP_
