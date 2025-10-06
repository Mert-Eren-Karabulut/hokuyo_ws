/*
 * Author       : Batuhan Vatan
 * Date         : 26 Aug '23
 * Description  : Utility functions for head_apf package
 */

#ifndef UTILITY_HPP
#define UTILITY_HPP

#include <iostream>
#include <string>
#include <utility>
#include <algorithm>
#include <vector>
#include <cmath>

// Include master ros pkgs
#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/opencv.hpp>

#include <geometry_msgs/Point.h>

#include "head_apf/definitions.hpp"

// Prototype of utility functions

void deg2rad(float deg, float *rad);
void rad2deg(float rad, float *deg);
// Overloads: Return type overloads for rad<->deg
float deg2rad(float deg);
float rad2deg(float rad);
// Overloads: Return type overloads for rad<->deg

void radpersec2RPM(float radpersec, float *RPM);
void RPM2radpersec(float RPM, float *radpersec);
// Overloads: Return type overloads for RPM<->rad/s
float RPM2radpersec(float RPM);
float radpersec2RPM(float radpersec);
// Overloads: Return type overloads for RPM<->rad/s

void degpersec2RPM(float deg, float *RPM);
void RPM2degpersec(float RPM, float *deg);
// Overloads: Return type overloads for degpersec<->RPM
float degpersec2RPM(float deg);
float RPM2degpersec(float RPM);
// Overloads: Return type overloads for degpersec<->RPM

void deg2Pulse(float deg, int *pulse);
void pulse2Deg(int pulse, float *deg);

void cartesian2Polar(cv::Point2i cart, cv::Point2f *polar);
void polar2Cartesian(cv::Point2f polar, cv::Point2i *cart);

int sign(float v);
// Prototype of utility functions

#endif // UTILITY_HPP