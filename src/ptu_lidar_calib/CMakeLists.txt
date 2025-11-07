cmake_minimum_required(VERSION 3.0.2)
project(ptu_lidar_calib)

add_definitions(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy sensor_msgs std_srvs tf2_ros tf2 tf2_geometry_msgs message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

add_service_files(
  FILES
  GoTo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs std_srvs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(ptu_lidar_calibrator src/ptu_lidar_calibrator.cpp)
target_link_libraries(ptu_lidar_calibrator
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)