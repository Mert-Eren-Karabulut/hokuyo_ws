# Quick Start Guide - Real Pan-Tilt System

## Overview
This guide provides step-by-step instructions to get your real pan-tilt laser scanning system running with proper TF transforms and voxelized pointcloud generation.

## Prerequisites

- ✅ Arduino with uploaded `servo_ros.ino` (with radian conversion)
- ✅ ROS workspace compiled (`catkin_make`)
- ✅ Hokuyo laser sensor connected and working
- ✅ Pan-tilt mechanism with encoders connected to Arduino

## System Architecture

```
Arduino → /joint_states (radians) → robot_state_publisher → TF Tree
                                            ↓
                                    laser_real_position
                                            ↓
                                    /output (pointcloud)
```

## Step-by-Step Launch

### 1. Start ROS Core
```bash
roscore
```

### 2. Connect Arduino (Terminal 2)
```bash
# Find your Arduino port
ls /dev/ttyACM*

# Start rosserial bridge
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

**Expected output:**
```
[INFO] [1234567890.123456]: ROS Serial Python Node
[INFO] [1234567890.123456]: Connecting to /dev/ttyACM0 at 57600 baud
[INFO] [1234567890.123456]: Setup subscriber on /joint_command [sensor_msgs/JointState]
[INFO] [1234567890.123456]: Setup publisher on /joint_states [sensor_msgs/JointState]
```

### 3. Verify Joint States (Terminal 3)
```bash
rostopic echo /joint_states
```

**Expected output (example at home position):**
```yaml
header: 
  seq: 42
  stamp: 
    secs: 1234567890
    nsecs: 123456789
  frame_id: ''
name: ['joint1', 'joint2', 'joint1Motor', 'joint2Motor', 'joint1Error', 'joint2Error', 'TiltPos', 'PanPos']
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []
```

**Note:** `position[0]` and `position[1]` should be in radians!

### 4. Launch the System (Terminal 4)
```bash
cd /home/isl9/dev/mert/hokuyo_ws
source devel/setup.bash
roslaunch hokuyo_go real_pantilt.launch
```

**This launch file starts:**
- `robot_state_publisher` (URDF → TF conversion)
- `laser_real_position` (scanning + pointcloud node)

**Expected output:**
```
[INFO] [1234567890.123456]: Loading robot model from parameter server
[INFO] [1234567890.123456]: TF Buffer initialized
[INFO] [1234567890.123456]: Waiting for joint states...
[INFO] [1234567890.123456]: Joint states received. Waiting for TF transforms...
[INFO] [1234567890.123456]: === SCANNING STARTED ===
```

### 5. Verify TF Tree (Terminal 5)
```bash
# Generate TF tree diagram
rosrun tf view_frames

# View the PDF
evince frames.pdf
```

**Expected TF tree:**
```
map
 └─ base_link
     └─ pan_link (joint2)
         └─ tilt_link (joint1)
             └─ hokuyo_base
                 └─ hokuyo_link
```

### 6. Test Transform (Terminal 5)
```bash
# Check transform from map to hokuyo_link
rosrun tf tf_echo map hokuyo_link
```

**Expected output:**
```
At time 1234567890.123
- Translation: [0.000, -0.064, 0.506]
- Rotation: in Quaternion [0.000, 0.707, 0.000, 0.707]
            in RPY (radian) [0.000, 1.571, 0.000]
            in RPY (degree) [0.000, 90.000, 0.000]
```

### 7. Monitor Topics

**Joint States (50Hz from Arduino):**
```bash
rostopic hz /joint_states
```

**Point Cloud Output:**
```bash
rostopic hz /output
```

**Laser Scan:**
```bash
rostopic hz /scan
```

## Visualize in RViz

### Terminal 6: Start RViz
```bash
rosrun rviz rviz
```

### RViz Configuration:

1. **Fixed Frame:** Set to `map`

2. **Add TF Display:**
   - Add → TF
   - Shows all coordinate frames

3. **Add LaserScan:**
   - Add → LaserScan
   - Topic: `/scan`
   - Frame: `hokuyo_link`

4. **Add PointCloud2:**
   - Add → PointCloud2
   - Topic: `/output`
   - Frame: `map`
   - Color Transformer: `AxisColor` or `Intensity`

5. **Add RobotModel (optional):**
   - Add → RobotModel
   - Shows URDF visualization

## Manual Control Test

### Send Joint Commands
```bash
# Move tilt to 45 degrees (0.785398 radians)
rostopic pub /joint_command sensor_msgs/JointState "
header:
  stamp: now
name: ['joint1', 'joint2']
position: [0.785398, 0.0]"

# Move pan to 30 degrees (0.523599 radians)
rostopic pub /joint_command sensor_msgs/JointState "
header:
  stamp: now
name: ['joint1', 'joint2']
position: [0.0, 0.523599]"

# Return home
rostopic pub /joint_command sensor_msgs/JointState "
header:
  stamp: now
name: ['joint1', 'joint2']
position: [0.0, 0.0]"
```

## Troubleshooting

### Problem: No /joint_states topic
**Solution:** Check Arduino connection and rosserial bridge
```bash
# Check if Arduino is detected
ls /dev/ttyACM*

# Check rosserial output for errors
```

### Problem: TF tree incomplete
**Solution:** Verify robot_state_publisher is running
```bash
rosnode list | grep robot_state_publisher
```

### Problem: Transform timeout errors
**Solution:** Check that joint names match between Arduino and URDF
```bash
# Check joint names from Arduino
rostopic echo /joint_states | grep name

# Should output: name: ['joint1', 'joint2', ...]
```

### Problem: Incorrect angles in TF
**Solution:** Verify Arduino is publishing radians (not degrees)
```bash
rostopic echo /joint_states | grep position
```
At 90°, position[0] or position[1] should be ~1.57 (radians), NOT 90.0 (degrees)

### Problem: No pointcloud output
**Solution:** 
1. Check laser is publishing: `rostopic echo /scan`
2. Check TF is working: `rosrun tf tf_echo map hokuyo_link`
3. Check laser_real_position logs for errors

## Expected Behavior

1. **Startup (0-5 sec):**
   - Arduino publishes /joint_states at 10Hz
   - robot_state_publisher starts publishing TF
   - laser_real_position waits for TF availability

2. **Scanning (5-25 sec):**
   - Pan-tilt follows parametric pattern
   - Laser points transformed to map frame
   - Points accumulated in 50mm voxel grid

3. **Completion (>25 sec):**
   - Pointcloud saved to timestamped PCD file
   - System returns to home position
   - Voxel grid published on /output topic

## Scan Output

**Location:** `/home/isl9/dev/mert/hokuyo_ws/`

**Filename format:** `pointcloud_YYYYMMDD_HHMMSS.pcd`

**Example:** `pointcloud_20251020_143025.pcd`

**View PCD file:**
```bash
# Install PCL tools if not already installed
sudo apt-get install pcl-tools

# View point cloud
pcl_viewer pointcloud_20251020_143025.pcd
```

## Key Files Reference

| File | Purpose |
|------|---------|
| `servo_ros.ino` | Arduino controller (radians I/O) |
| `real_pantilt.urdf` | Robot kinematics (joint1, joint2) |
| `real_pantilt.launch` | Launch robot_state_publisher + scanning |
| `laser_real_position.cpp` | Scanning node with TF + voxelization |

## Useful Commands

```bash
# View all topics
rostopic list

# Monitor joint state values
rostopic echo /joint_states

# Check TF frames
rosrun tf tf_monitor

# View computation graph
rqt_graph

# Record data
rosbag record /joint_states /scan /output /tf /tf_static
```

## Next Steps

Once the system is working:
1. Tune PID parameters in Arduino code for smoother motion
2. Adjust voxel size in launch file (default 50mm)
3. Modify scan_duration for longer/shorter scans
4. Experiment with delta_1 parameter for different scan patterns
5. Post-process PCD files with PCL for filtering/segmentation

## Support

For issues, check:
- `SYSTEM_INTEGRATION.md` - Full system architecture
- `UNIT_CONVERSION_UPDATE.md` - Details on radian conversion
- `TF_SETUP_GUIDE.md` - Alternative TF publishing methods
- `POINTCLOUD_MIGRATION_SUMMARY.md` - Voxelization details
