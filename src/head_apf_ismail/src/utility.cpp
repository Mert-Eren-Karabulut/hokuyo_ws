/*
 * Author       : Batuhan Vatan
 * Date         : 26 Aug '23
 * Description  : Utility functions for head_apf package
 */

#include "head_apf/utility.hpp"

// Implementation of utility functions
void deg2rad(float deg, float *rad)
{
    *rad = deg * (PI / 180.0);
}

void rad2deg(float rad, float *deg)
{
    *deg = rad * (180.0 / PI);
}

// Overloads
float deg2rad(float deg)
{
    return deg * (PI / 180.0);
}
float rad2deg(float rad)
{
    return rad * (180.0 / PI);
}
// Overloads

void radpersec2RPM(float radpersec, float *RPM)
{
    *RPM = radpersec * (60.0 / (2 * PI));
}

void RPM2radpersec(float RPM, float *radpersec)
{
    *radpersec = RPM * (2 * PI / 60.0);
}

// Overloads
float RPM2radpersec(float RPM)
{
    return RPM * ((2 * PI) / 60.0);
}

float radpersec2RPM(float radpersec)
{
    return radpersec * (60.0 / (2 * PI));
}
// Overloads

void degpersec2RPM(float deg, float *RPM)
{
    *RPM = deg * (60.0 / 360.0);
}
void RPM2degpersec(float RPM, float *deg)
{
    *deg = RPM * (360.0 / 60.0);
}

// Overloads
float degpersec2RPM(float deg)
{
    return deg * (360.0 / 60.0);
}
float RPM2degpersec(float RPM)
{
    return RPM * (60.0 / 360.0);
}
// Overloads

void deg2Pulse(float deg, int *pulse)
{
    *pulse = round((MOTOR_MAX_PULSE / MOTOR_MAX_ANGLE) * deg);
}

void pulse2Deg(int pulse, float *deg)
{
    *deg = (MOTOR_MAX_ANGLE / MOTOR_MAX_PULSE) * pulse;
}

// void cartesian2Polar(cv::Point2i cart, cv::Point2f *polar)
// {
//     polar->x = sign(cart.x - CAMERA_WIDTH / 2) * atan(abs(cart.x - CAMERA_WIDTH / 2) * tan(PI * CAMERA_FOV_X / 360.0) * 2 / CAMERA_WIDTH) * 180.0 / PI;
//     polar->y = -sign(-cart.y + CAMERA_HEIGHT / 2) * atan(abs(cart.y - CAMERA_HEIGHT / 2) * tan(PI * CAMERA_FOV_Y / 360.0) * 2 / CAMERA_HEIGHT) * 180.0 / PI;
// }

// void polar2Cartesian(cv::Point2f polar, cv::Point2i *cart)
// {
//     cart->x = (CAMERA_WIDTH * (tan(polar.x * PI / 180) / (tan(CAMERA_FOV_X * PI / 360.0))) / 2 + CAMERA_WIDTH / 2);
//     cart->y = (CAMERA_HEIGHT * (tan(polar.y * PI / 180) / (tan(CAMERA_FOV_Y * PI / 360.0))) / 2 + CAMERA_HEIGHT / 2);
// }

int sign(float v)
{
    return v > 0 ? 1 : (v < 0 ? -1 : 0);
}

// Implementation of utility functions
