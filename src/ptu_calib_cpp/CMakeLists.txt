cmake_minimum_required(VERSION 3.1.3)
project(ptu_calib_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra)

# ── ROS packages ──────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  gazebo_msgs
)

# ── System dependencies ──────────────────────────────────────────────
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs
                 visualization_msgs tf2_ros tf2_eigen tf2_geometry_msgs
                 gazebo_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# ── Executables ───────────────────────────────────────────────────────

# 1. Data acquisition node
add_executable(acquire_node
  src/acquire_node.cpp
)
target_link_libraries(acquire_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

# 2. Calibration solver (offline, reads .csv written by acquire_node)
add_executable(solve_node
  src/solve_node.cpp
)
target_link_libraries(solve_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

# 3. Full pipeline (acquire → solve pan → solve tilt → emit URDF)
add_executable(pipeline_node
  src/pipeline_node.cpp
)
target_link_libraries(pipeline_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
)

# 4. Sim joint controller (Gazebo PD controller, same /joint_command interface)
add_executable(sim_joint_controller
  src/sim_joint_controller.cpp
)
target_link_libraries(sim_joint_controller
  ${catkin_LIBRARIES}
)

# ── Install ───────────────────────────────────────────────────────────
install(TARGETS acquire_node solve_node pipeline_node sim_joint_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
