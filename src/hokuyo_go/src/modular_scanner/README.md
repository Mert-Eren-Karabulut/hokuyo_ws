# Laser Scanner Node - Modular Architecture

## Overview

The laser scanner node has been refactored into a clean, modular architecture for better maintainability and reusability.

## Project Structure

```
hokuyo_go/
├── include/hokuyo_go/modular_scanner/
│   ├── voxel_grid.h            # Voxel grid data structures and manager
│   ├── parametric_scanner.h    # Parametric scanning pattern generator
│   └── laser_scanner_node.h    # Main ROS node class
├── src/modular_scanner/
│   ├── voxel_grid.cpp          # Voxel grid implementation
│   ├── parametric_scanner.cpp  # Parametric scanner implementation
│   ├── laser_scanner_node.cpp  # Main node logic
│   ├── laser_real_position_main.cpp  # Entry point
│   └── README.md               # This file
├── src/
│   └── laser_real_position.cpp # Legacy monolithic version (kept for reference)
```

## Module Descriptions

### 1. **VoxelGrid Module** (`voxel_grid.h/cpp`)
**Purpose:** Manages hierarchical voxelization for point cloud generation.

**Key Components:**
- `VoxelPoint`: Individual point with averaging and subdivision level tracking
- `SubVoxel`: Hierarchical octree subdivision structure
- `ParentVoxel`: Top-level voxel container managing subdivisions
- `VoxelGrid`: Main manager class for the entire voxel grid

**Features:**
- Adaptive multi-resolution voxelization (up to 3 subdivision levels)
- Arithmetic averaging for point consolidation
- Color-coded visualization by subdivision level
- Configurable subdivision threshold

### 2. **ParametricScanner Module** (`parametric_scanner.h/cpp`)
**Purpose:** Generates smooth parametric scanning trajectories.

**Key Components:**
- Constant velocity trajectory generation
- Pan/tilt offset management for focus point tracking
- Parametric functions f1(t) and f2(t) for smooth motion

**Features:**
- Irrational frequency for non-repeating patterns
- Configurable pan/tilt limits
- Real-time parameter time updates
- Pattern center adjustment via IK

### 3. **LaserScannerNode Module** (`laser_scanner_node.h/cpp`)
**Purpose:** Main ROS node coordinating all components.

**Key Responsibilities:**
- ROS communication (subscribers/publishers)
- TF2 transform management
- Laser scan processing and voxelization
- Joint state tracking and velocity calculation
- Focus point IK solver
- Point cloud generation and saving

**ROS Interface:**
- **Subscribers:**
  - `/joint_states`: Joint positions from Arduino
  - `/scan`: Laser scan data
  - `/clicked_point`: Focus point for IK

- **Publishers:**
  - `/joint_command`: Target joint positions
  - `/output`: Voxelized point cloud
  - `/debug_marker`: Visualization markers

### 4. **Main Entry Point** (`laser_real_position_main.cpp`)
**Purpose:** Minimal main function instantiating the node.

## Build System

The CMakeLists.txt has been updated to build all modules:

```cmake
add_executable(laser_real_position 
  src/modular_scanner/laser_real_position_main.cpp
  src/modular_scanner/laser_scanner_node.cpp
  src/modular_scanner/parametric_scanner.cpp
  src/modular_scanner/voxel_grid.cpp
)
target_link_libraries(laser_real_position ${catkin_LIBRARIES})
```

The legacy monolithic version is still available as `laser_real_position_legacy` for comparison.

## Building

```bash
cd ~/hokuyo_ws
catkin_make
source devel/setup.bash
```

## Running

```bash
roslaunch hokuyo_go real_pantilt.launch
```

## Configuration Parameters

All parameters can be set via ROS param server or launch file:

```xml
<node name="laser_real_position" pkg="hokuyo_go" type="laser_real_position">
  <param name="target_velocity" value="1.0"/>        <!-- rad/s -->
  <param name="scan_duration" value="60.0"/>          <!-- seconds -->
  <param name="pan_limit_deg" value="30.0"/>          <!-- degrees -->
  <param name="tilt_limit_deg" value="20.0"/>         <!-- degrees -->
  <param name="voxel_size" value="0.05"/>             <!-- meters -->
  <param name="subvoxel_threshold" value="0.35"/>     <!-- 0.0-1.0 -->
</node>
```

## Benefits of Modular Architecture

1. **Maintainability**: Each module has a single, well-defined responsibility
2. **Testability**: Individual components can be unit tested in isolation
3. **Reusability**: Modules can be used in other projects (e.g., simulation)
4. **Readability**: Shorter files with clear interfaces
5. **Extensibility**: Easy to add new features without affecting other modules

## Migration Notes

The refactored code maintains **100% functional equivalence** with the original monolithic version. All algorithms, parameters, and behaviors are preserved.

### Key Differences:
- **Before**: Single 1000+ line file
- **After**: 7 organized files (3 headers + 4 implementations)

### What Was Changed:
- Code organization only
- Added namespace `hokuyo_go::`
- Encapsulated globals into class members
- No algorithmic changes

## Future Enhancements

With the modular structure, these improvements are now easier:

1. **Unit Tests**: Add `test/` directory with Google Test
2. **Simulation Mode**: Swap `LaserScannerNode` with `SimulatedScannerNode`
3. **Multiple Scanners**: Instantiate multiple `ParametricScanner` objects
4. **Alternative Voxelization**: Implement different `VoxelGrid` strategies
5. **ROS2 Port**: Update only ROS-specific code in `LaserScannerNode`

## Debugging

Each module logs with appropriate ROS_INFO/WARN/ERROR messages. Use:

```bash
rosrun rqt_console rqt_console  # View logs
rosrun rqt_graph rqt_graph      # Visualize node graph
```

## Contact

For questions or issues with the refactored code, please refer to the original implementation in `laser_real_position.cpp` for algorithmic details.
