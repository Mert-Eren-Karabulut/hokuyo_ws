# Laser Real System Migration Guide

## Overview
Migration from velocity-based control (`laser_real.cpp`) to position-based control (`laser_real_position.cpp`) to match the Arduino QuickPID position controller.

## Key Changes

### 1. **Control Paradigm Shift**
| Aspect | Old (laser_real.cpp) | New (laser_real_position.cpp) |
|--------|---------------------|-------------------------------|
| **Control Type** | Velocity-based | **Position-based** |
| **Command Topic** | `/cmd_vel` (Twist) | **`/joint_command` (JointState)** |
| **Command Data** | `linear.x` = tilt velocity<br>`linear.y` = pan velocity | `position[0]` = tilt target<br>`position[1]` = pan target |
| **Motion Pattern** | Simple back-and-forth | **Parametric non-overlapping scan** |

### 2. **Parametric Scanning**
Now uses the **same parametric equations** as `vel_publisher.cpp`:

```cpp
// Pan (yaw): sinusoidal motion
f1(t) = delta_1 * sin(t)

// Tilt: non-repetitive pattern with irrational frequency
f2(t) = (π/2) * (cos((3 + √2/100)t) + 1) / 2 - π/4
```

**Benefits:**
- ✅ Non-overlapping coverage (irrational frequency component)
- ✅ Smooth continuous motion
- ✅ Constant velocity scanning
- ✅ Matches simulation behavior

### 3. **Simplified State Machine**
Old system had complex multi-state logic. New system:
- Simple boolean flag: `scanning_active`
- Duration-based scanning: configurable via `scan_duration` parameter
- Automatic return to home when complete

### 4. **ROS Parameter Configuration**
```bash
# Launch with custom parameters
rosrun hokuyo_go laser_real_position _target_velocity:=0.5 _scan_duration:=30.0
```

Available parameters:
- `target_velocity` - Scanning speed in rad/s (default: 0.5)
- `pan_limit_deg` - Pan range in degrees (default: ±60°)
- `tilt_limit_deg` - Tilt range in degrees (default: ±45°)
- `scan_duration` - Scan time in seconds (default: 30.0)
- `scan_count` - Target number of scans (default: 2000)

## Architecture Comparison

### Old System Flow (laser_real.cpp)
```
1. State machine decides motion phase
2. Calculate velocity commands
3. Publish Twist to /cmd_vel
4. Arduino converts velocity to position (custom PID)
5. Collect pointcloud during specific states
```

### New System Flow (laser_real_position.cpp)
```
1. Update parametric time (constant velocity)
2. Calculate position targets from f1(t), f2(t)
3. Publish JointState to /joint_command
4. Arduino QuickPID tracks position
5. Collect pointcloud throughout scan
```

## Topic Communication

### Subscribe (Inputs)
```
/joint_states (sensor_msgs/JointState)
  - From: Arduino
  - Data: position[0] = tilt, position[1] = pan
  - Rate: ~10Hz

/scan (sensor_msgs/LaserScan)
  - From: Hokuyo LIDAR
  - Data: range measurements
  - Rate: ~40Hz
```

### Publish (Outputs)
```
/joint_command (sensor_msgs/JointState)
  - To: Arduino
  - Data: position[0] = tilt_target, position[1] = pan_target
  - Rate: 50Hz

/output (sensor_msgs/PointCloud2)
  - To: Visualization/Processing
  - Data: 3D point cloud
  - Rate: 50Hz
```

## Arduino Compatibility

### Joint Naming Convention
Both systems use consistent naming:
- Arduino publishes: `joint1` (tilt), `joint2` (pan)
- C++ node commands: `joint1` (tilt), `joint2` (pan)

### Position Units
- **Radians** for all angle communications
- Arduino internally uses degrees (converted via `DEGREES_PER_COUNT`)
- Limits: ±180° (±π radians)

## Pointcloud Generation

### Maintained Features
- ✅ Same kinematic transformation equations
- ✅ Same coordinate frame conventions
- ✅ Configurable scan count
- ✅ Automatic PCD file saving
- ✅ Progress reporting

### Improvements
- ✅ Timestamped filenames
- ✅ Continuous collection during scan
- ✅ Better progress feedback
- ✅ Cleaner start/stop logic

## Build and Run

### CMakeLists.txt
Add to your `hokuyo_go/CMakeLists.txt`:
```cmake
add_executable(laser_real_position src/laser_real_position.cpp)
target_link_libraries(laser_real_position ${catkin_LIBRARIES})
```

### Build
```bash
cd ~/hokuyo_ws
catkin_make
source devel/setup.bash
```

### Launch System
```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Start rosserial for Arduino
rosrun rosserial_python serial_node.py /dev/ttyACM0

# Terminal 3: Start Hokuyo node
rosrun urg_node urg_node

# Terminal 4: Start laser scanner
rosrun hokuyo_go laser_real_position
```

### Visualization
```bash
# View pointcloud in RViz
rviz

# Or monitor topics
rostopic echo /joint_states
rostopic echo /joint_command
```

## Testing Procedure

1. **Verify Arduino Communication**
   ```bash
   rostopic echo /joint_states
   # Should see position[0] and position[1] updating
   ```

2. **Test Position Commands**
   ```bash
   # Manual position command
   rostopic pub /joint_command sensor_msgs/JointState "{position: [0.5, 0.3]}"
   # Pan-tilt should move to tilt=0.5rad, pan=0.3rad
   ```

3. **Start Scanning**
   ```bash
   rosrun hokuyo_go laser_real_position
   # Wait for "SCANNING STARTED" message
   # Observe parametric motion pattern
   ```

4. **Verify Pointcloud**
   ```bash
   # Check pointcloud output
   rostopic hz /output
   # Should be ~50Hz
   
   # After scan completes, verify PCD file
   ls -lh pointcloud_*.pcd
   ```

## Tuning Parameters

### If motion is too slow:
```bash
rosrun hokuyo_go laser_real_position _target_velocity:=1.0
```

### If motion is too fast (oscillations):
```bash
rosrun hokuyo_go laser_real_position _target_velocity:=0.3
```

### Adjust scanning range:
```bash
rosrun hokuyo_go laser_real_position _pan_limit_deg:=45 _tilt_limit_deg:=30
```

### Longer/shorter scans:
```bash
rosrun hokuyo_go laser_real_position _scan_duration:=60.0 _scan_count:=4000
```

## Troubleshooting

### Motors not moving
- Check `/joint_states` is publishing from Arduino
- Verify `/joint_command` is being published
- Check Arduino PID gains in servo_ros.ino
- Ensure position limits are reasonable

### Jerky motion
- Reduce `target_velocity` parameter
- Increase Arduino PID Kd gain
- Check for ROS communication delays

### Incomplete pointcloud
- Increase `scan_duration`
- Increase `scan_count`
- Check laser `/scan` topic frequency

### Position overshoot
- Tune Arduino PID gains (especially Kd)
- Reduce `target_velocity`
- Check for mechanical backlash

## Migration Checklist

- [ ] Build new `laser_real_position` node
- [ ] Upload updated `servo_ros.ino` to Arduino
- [ ] Test rosserial communication
- [ ] Verify joint state publishing
- [ ] Test manual position commands
- [ ] Run full parametric scan
- [ ] Verify pointcloud quality
- [ ] Compare with simulation results
- [ ] Document optimal parameter values

## Next Steps

After successful migration:
1. Compare pointcloud quality with old system
2. Optimize PID gains for your specific hardware
3. Tune scanning parameters for coverage
4. Implement automatic rescan capability
5. Add real-time pointcloud filtering
