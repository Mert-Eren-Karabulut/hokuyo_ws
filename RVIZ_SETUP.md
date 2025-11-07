# RViz Visualization Setup Guide

## Problem
Error: "Received a joint state msg with different joint names and efforts size!"

This error appears when using the wrong RViz display type for robot visualization.

## Solution: Use RobotModel Display

### Step-by-Step Setup:

1. **Open RViz:**
   ```bash
   rosrun rviz rviz
   ```

2. **Set Fixed Frame:**
   - In the left panel, find "Global Options"
   - Set "Fixed Frame" to `map`

3. **Add RobotModel Display:**
   - Click "Add" button at bottom left
   - Select "RobotModel"
   - Click OK

4. **Configure RobotModel:**
   - The robot should appear automatically (reads from `/robot_description` parameter)
   - If not visible, check:
     - "Robot Description" should be set to `robot_description`
     - Expand the RobotModel tree to see all links
     - Make sure all links show green checkmarks

5. **Add TF Display (Optional but Recommended):**
   - Click "Add" button
   - Select "TF"
   - Click OK
   - This will show all coordinate frame axes
   - You can adjust "Marker Scale" to make them smaller/larger

6. **Add Axes Display for Map Frame:**
   - Click "Add" button
   - Select "Axes"
   - This shows the map frame origin

## Expected Visualization

You should see:
- ✅ Base platform (cyan box)
- ✅ Pan link (yellow box) - rotates around Z axis
- ✅ Tilt link (cyan bar) - tilts up/down
- ✅ Hokuyo base plate (yellow flat box)
- ✅ Hokuyo sensor (cyan box)
- ✅ TF axes at each joint showing orientations

## Testing Movement

### Manual Test Command:
```bash
# Move pan to 45 degrees (0.785 radians)
rostopic pub /joint_command sensor_msgs/JointState "
header:
  stamp: now
name: ['joint1', 'joint2']
position: [0.0, 0.785398]"

# Move tilt to 30 degrees (0.523 radians)
rostopic pub /joint_command sensor_msgs/JointState "
header:
  stamp: now
name: ['joint1', 'joint2']
position: [0.523599, 0.0]"

# Return to home
rostopic pub /joint_command sensor_msgs/JointState "
header:
  stamp: now
name: ['joint1', 'joint2']
position: [0.0, 0.0]"
```

Watch the robot in RViz - it should move smoothly!

## Troubleshooting

### Robot Not Appearing:
```bash
# Check if URDF is loaded
rosparam get /robot_description | head -n 20

# Check TF tree
rosrun tf view_frames
evince frames.pdf

# Verify joint_states are publishing
rostopic hz /joint_states
```

### Robot Appears Gray/Broken:
- This means TF transforms are not being received
- Check that `robot_state_publisher` is running:
  ```bash
  rosnode list | grep robot_state_publisher
  ```

### Wrong DON'T Use These Displays:

❌ **JointState** display with "Topic" set to `/joint_states`
- This is for plotting joint values over time (graphs)
- Not for 3D visualization
- Causes the "different size" error

❌ **InteractiveMarkers** or **Marker** displays
- These are for custom visualization markers
- Not for URDF robot models

## Saving RViz Configuration

Once you have it working:

1. **File → Save Config As...**
2. Save to: `/home/isl9/dev/mert/hokuyo_ws/src/hokuyo_go/rviz/real_pantilt.rviz`
3. Update launch file to auto-load this config:
   ```xml
   <node name="rviz" pkg="rviz" type="rviz" 
         args="-d $(find hokuyo_go)/rviz/real_pantilt.rviz" />
   ```

## Quick Test Sequence

1. Verify system is running:
   ```bash
   # Should show: roscore, serial_node, robot_state_publisher, laser_real_position
   rosnode list
   ```

2. Check topics:
   ```bash
   rostopic list | grep -E "joint|tf"
   # Should show: /joint_command, /joint_states, /tf, /tf_static
   ```

3. Open RViz with correct setup
4. Send test command
5. Watch robot move in RViz!

## Advanced: Adding LaserScan Visualization

Once you have the laser connected:

1. **Add LaserScan Display:**
   - Click "Add"
   - Select "LaserScan"
   - Topic: `/scan`
   - Size (m): 0.01 (makes points visible)
   - Color: By intensity or by axis

2. **The laser scan should appear in the hokuyo_link frame**
   - Points will move with the pan-tilt mechanism
   - Shows real-time scan data

3. **Add PointCloud2 for Accumulated Points:**
   - Click "Add"
   - Select "PointCloud2"
   - Topic: `/output`
   - This shows the voxelized global pointcloud
   - Should be in `map` frame
