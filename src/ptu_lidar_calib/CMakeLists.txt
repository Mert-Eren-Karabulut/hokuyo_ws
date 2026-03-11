cmake_minimum_required(VERSION 3.0.2)
project(ptu_lidar_calib)

find_package(catkin REQUIRED COMPONENTS
  rospy sensor_msgs std_srvs tf2_ros message_generation
)

add_service_files(
  FILES
  GoTo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs std_srvs message_runtime
)

# Install Python calibration scripts
catkin_install_python(PROGRAMS
  scripts/calibration_acquisition.py
  scripts/calibration_solver.py
  scripts/calibration_validation.py
  scripts/calibration_pipeline.py
  scripts/ptu_command_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)