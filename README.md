# Low Cost Lidar ROS Workspace

This ROS workspace contains packages for low-cost LiDAR-based robotics applications, including simulation, navigation, and point cloud processing.

## Repository Structure

### Submodules
- **robot_state_publisher** - Official ROS package for publishing robot state information
- **aws-robomaker-small-house-world** - AWS RoboMaker small house world for Gazebo simulation

### Custom ROS Packages

- **gazebo_sim** - Gazebo simulation configurations and robot models
- **head_apf_ismail** - Head movement with artificial potential fields
- **hokuyo_go** - LiDAR data processing and point cloud generation
- **hokuyo_plus** - MoveIt! configuration and motion planning
- **segmentation** - Point cloud segmentation algorithms
- **zed_params** - ZED camera parameters and configuration

## Setup Instructions

1. Clone this repository:
   ```bash
   git clone <your-repository-url>
   cd hokuyo_ws
   ```

2. Initialize and update submodules:
   ```bash
   git submodule init
   git submodule update
   ```

3. Install dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. Build the workspace:
   ```bash
   catkin_make
   ```

5. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

## Usage

### Simulation
Launch the small house simulation:
```bash
roslaunch hokuyo_plus small_house_turtle.launch
```

### LiDAR Processing
Start the adaptive voxel scanning:
```bash
roslaunch hokuyo_go adaptive_voxel_scan.launch
```

### Visualization
Open RViz for visualization:
```bash
rviz
```

## Requirements

- ROS Noetic
- Gazebo
- MoveIt!
- PCL (Point Cloud Library)
- OpenCV

## Contributing

This workspace is part of a graduation project focused on low-cost LiDAR applications in robotics.