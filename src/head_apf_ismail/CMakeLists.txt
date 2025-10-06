cmake_minimum_required(VERSION 3.10.2)
project(head_apfrl_ismail)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  roslaunch
  geometry_msgs
  sensor_msgs
  control_msgs
  std_msgs
)
#find_package(OpenCV REQUIRED)


# configure the necessary common CMake environment variables
# needed to include and link the OpenCV program into this
# demo project, namely OpenCV_INCLUDE_DIRS and OpenCV_LIBS
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV 4 REQUIRED)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# tell the build to include the headers from OpenCV
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${OpenCV_LIBS} )
include_directories( ${OpenCV_DIR} )

set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake #inside theres findSFML.cmake required for compilation
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp roslaunch cv_bridge geometry_msgs roslaunch sensor_msgs std_msgs
  # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/opencv4/opencv2
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/semprob_head_apf.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
file(GLOB SOURCES
  src/*.cpp
  head_apf/*.hpp
)

add_executable(${PROJECT_NAME}_node 
  ${SOURCES}
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

roslaunch_add_file_check(launch)
