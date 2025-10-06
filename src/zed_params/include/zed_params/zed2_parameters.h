/*
 * Author       : Batuhan Vatan
 * Date         : 31 Oct '23
 * Description  : ZED2 camera parameters
 */

#ifndef __ZED_CAMERA_PARAMETERS_H__
#define __ZED_CAMERA_PARAMETERS_H__
#pragma once

#include <string>

// ZED2 camera parameters: After installing ZED SDK,
// you can find the camera parameters in the following path:
// /usr/local/zed/settings/SN<serial_number>.conf
// Or address the camera info topics

// Example:  static ZED2Parameters &ZED2Params = ZED2Parameters::getInstance();

struct ZED2Parameters
{
  // General Properties
  std::string camera_name;
  std::string grab_resolution;
  std::string frame_rate;

  // Camera focal length (in mm)
  float focal_length = 0.0f;

  // Camera resolution
  float width = 0.0f;
  float height = 0.0f;

  // Camera field of view
  float rgb_fov_x = 0.0f;
  float rgb_fov_y = 0.0f;
  float depth_fov_x = 0.0f;
  float depth_fov_y = 0.0f;

  // Camera focal length
  float fx = 0.0f;
  float fy = 0.0f;

  // Camera center point
  float cx = 0.0f;
  float cy = 0.0f;

  // Camera distortion coefficients
  float k1 = 0.0f;
  float k2 = 0.0f;
  float k3 = 0.0f;
  float p1 = 0.0f;
  float p2 = 0.0f;

  // Camera baseline
  float baseline = 0.0f;

  float TY = 0.0f;
  float TZ = 0.0f;

  float CV = 0.0f;
  float RX = 0.0f;
  float RZ = 0.0f;

  // Camera depth range
  float min_depth = 0.0f;
  float max_depth = 0.0f;

  // Depth-related properties
  float depth_resample_factor = 0.0f;
  float depth_downsample_factor = 0.0f;
  int depth_confidence = 0;
  int depth_texture_conf = 0;
  double point_cloud_freq = 0.0;

  // Camera video properties
  int brightness = 0;
  int contrast = 0;
  int hue = 0;
  int saturation = 0;
  int sharpness = 0;
  int gamma = 0;
  int gain = 0;
  int exposure = 0;
  int whitebalance_temperature = 0;

private:
  // Private constructor to initialize the properties
  ZED2Parameters() : camera_name("ZED2"),
                     grab_resolution("WVGA"),
                     frame_rate("20"),
                     focal_length(0.0029f), // 2.9 mm
                     width(640.0),
                     height(360.0),
                     rgb_fov_x(110.0f),
                     rgb_fov_y(70.0f),
                     depth_fov_x(110.0f),
                     depth_fov_y(70.0f),
                     fx(264.1356201171875f),
                     fy(264.1356201171875f),
                     cx(338.2852783203125f),
                     cy(181.83871459960938f),
                     k1(-0.04253065f),
                     k2(0.010449155f),
                     k3(-0.004684705f),
                     p1(-0.000263191f),
                     p2(0.000435194f),
                     baseline(0.12f),
                     TY(0.10937f),
                     TZ(-0.0767039f),
                     CV(-0.00328314f),
                     RX(-0.00216263f),
                     RZ(-0.00117585f),
                     min_depth(0.3f),
                     max_depth(20.0f),
                     depth_resample_factor(0.5f),
                     depth_downsample_factor(0.5f),
                     depth_confidence(30),
                     depth_texture_conf(100),
                     point_cloud_freq(10.0f),
                     brightness(4),
                     contrast(4),
                     hue(0),
                     saturation(4),
                     sharpness(4),
                     gamma(8),
                     gain(100),
                     exposure(100),
                     whitebalance_temperature(42)
  {
    // Constructor body
  }

public:
  // Static method to get the singleton instance
  static ZED2Parameters &getInstance()
  {
    static ZED2Parameters instance;
    return instance;
  }
};

#endif // __ZED_CAMERA_PARAMETERS_H__

// From lifelong learning
// #define RGB_FOCAL_X 554.254691191187
// #define RGB_FOCAL_Y 554.254691191187
// #define RGB_CENTER_X 320.5
// #define RGB_CENTER_Y 240.5
// #define DEPTH_FOCAL_X 554.254691191187
// #define DEPTH_FOCAL_Y 554.254691191187
// #define DEPTH_CENTER_X 320.5
// #define DEPTH_CENTER_Y 240.5