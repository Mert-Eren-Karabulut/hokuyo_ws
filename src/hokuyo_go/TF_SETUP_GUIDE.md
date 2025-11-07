# TF-Based Pointcloud Migration for Real System

## Overview
The `laser_real_position.cpp` has been updated to use **TF transforms and voxelization** instead of hardcoded kinematics. This matches the approach used in `laser_to_pointcloud.cpp` for simulation.

## Key Changes

### 1. **Eliminated Hardcoded Kinematics**
**Old approach:**
```cpp
// Hardcoded transformation equations
x[i] = xx * (cos(panangle) * cos(tiltangle) - sin(panangle) * sin(tiltangle)) 
     + d_offset * (...) + l_offset * (...) - x1 * (...)
```

**New approach:**
```cpp
// TF-based transformation
tf_buffer->transform(point_hokuyo, point_global, "map", ros::Duration(0.1));
```

### 2. **Added Voxelization**
- **50mm global voxel grid** for consistent point cloud quality
- **Arithmetic averaging** of overlapping points
- **Memory efficient** - only stores one point per voxel
- **Removes redundancy** from overlapping scans

### 3. **TF Transform Pipeline**
```
hokuyo_link → tilt_link → pan_link → base_link → map
```

Points are transformed from `hokuyo_link` frame directly to `map` frame using ROS TF.

## Setting Up TF for Real System

Since you don't have URDF/xacro for the real robot, you need to publish TF transforms manually.

### Option 1: Static Transform Publisher (Recommended for Testing)

Create a launch file `robot_tf.launch`:

```xml
<launch>
    <!-- Static transforms for pan-tilt mechanism -->
    
    <!-- Base link is the reference frame -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="map_to_base" 
          args="0 0 0 0 0 0 map base_link" />
    
    <!-- Pan joint (rotates around Z axis) -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_pan" 
          args="0 0 0.15 0 0 0 base_link pan_link" />
    
    <!-- Note: pan and tilt dynamic transforms will come from joint state publisher -->
    
    <!-- Hokuyo mounted on tilt mechanism -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="tilt_to_hokuyo" 
          args="0.106 0 0.037 0 0 0 tilt_link hokuyo_link" />
    
    <!-- Publish joint states as TF transforms -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
        <param name="publish_frequency" value="50.0"/>
    </node>
</launch>
```

### Option 2: Create Minimal URDF (Better Long-term Solution)

Create `real_pantilt.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot name="real_pantilt" xmlns:xacro="http://www.ros.org/wiki/xacro">
    
    <!-- Base link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.05"/>
            </geometry>
        </visual>
    </link>
    
    <!-- Pan link -->
    <link name="pan_link">
        <visual>
            <geometry>
                <cylinder radius="0.03" length="0.08"/>
            </geometry>
        </visual>
    </link>
    
    <!-- Pan joint (revolute around Z) -->
    <joint name="pan_joint" type="revolute">
        <parent link="base_link"/>
        <child link="pan_link"/>
        <origin xyz="0 0 0.15" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
    </joint>
    
    <!-- Tilt link -->
    <link name="tilt_link">
        <visual>
            <geometry>
                <box size="0.05 0.08 0.03"/>
            </geometry>
        </visual>
    </link>
    
    <!-- Tilt joint (revolute around Y) -->
    <joint name="tilt_joint" type="revolute">
        <parent link="pan_link"/>
        <child link="tilt_link"/>
        <origin xyz="0 0 0.08" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-0.79" upper="0.79" effort="100" velocity="1.0"/>
    </joint>
    
    <!-- Hokuyo link -->
    <link name="hokuyo_link">
        <visual>
            <geometry>
                <box size="0.05 0.05 0.07"/>
            </geometry>
        </visual>
    </link>
    
    <!-- Hokuyo mounting -->
    <joint name="hokuyo_joint" type="fixed">
        <parent link="tilt_link"/>
        <child link="hokuyo_link"/>
        <origin xyz="0.106 0 0.037" rpy="0 0 0"/>
    </joint>
    
    <!-- Map frame (world reference) -->
    <link name="map"/>
    
    <joint name="map_to_base" type="fixed">
        <parent link="map"/>
        <child link="base_link"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>
    
</robot>
```

Launch with:
```xml
<launch>
    <!-- Load URDF -->
    <param name="robot_description" command="cat $(find hokuyo_go)/urdf/real_pantilt.urdf.xacro" />
    
    <!-- Robot state publisher (publishes TF from joint states) -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher">
        <param name="publish_frequency" value="50.0"/>
    </node>
</launch>
```

### Option 3: Python TF Broadcaster (Quick Debug)

Create `broadcast_pantilt_tf.py`:

```python
#!/usr/bin/env python
import rospy
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import math

class PanTiltTFBroadcaster:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        
        # Publish static transforms
        self.publish_static_transforms()
        
        # Subscribe to joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        
        self.pan_angle = 0.0
        self.tilt_angle = 0.0
    
    def publish_static_transforms(self):
        transforms = []
        
        # map -> base_link
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # tilt_link -> hokuyo_link
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "tilt_link"
        t.child_frame_id = "hokuyo_link"
        t.transform.translation.x = 0.106
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.037
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        self.static_br.sendTransform(transforms)
    
    def joint_callback(self, msg):
        # Assume position[0] = tilt, position[1] = pan from Arduino
        if len(msg.position) >= 2:
            self.tilt_angle = math.radians(msg.position[0])  # Convert to radians if needed
            self.pan_angle = math.radians(msg.position[1])
            
            self.publish_dynamic_transforms()
    
    def publish_dynamic_transforms(self):
        # base_link -> pan_link
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "pan_link"
        t.transform.translation.z = 0.15
        t.transform.rotation = self.quaternion_from_euler(0, 0, self.pan_angle)
        self.br.sendTransform(t)
        
        # pan_link -> tilt_link
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "pan_link"
        t.child_frame_id = "tilt_link"
        t.transform.translation.z = 0.08
        t.transform.rotation = self.quaternion_from_euler(0, self.tilt_angle, 0)
        self.br.sendTransform(t)
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        from geometry_msgs.msg import Quaternion
        import tf.transformations as tft
        q = tft.quaternion_from_euler(roll, pitch, yaw)
        quat = Quaternion()
        quat.x = q[0]
        quat.y = q[1]
        quat.z = q[2]
        quat.w = q[3]
        return quat

if __name__ == '__main__':
    rospy.init_node('pantilt_tf_broadcaster')
    broadcaster = PanTiltTFBroadcaster()
    rospy.spin()
```

## Verifying TF Setup

### 1. Check TF Tree
```bash
rosrun tf view_frames
evince frames.pdf
```

Should show: `map → base_link → pan_link → tilt_link → hokuyo_link`

### 2. Check Transform Availability
```bash
rosrun tf tf_echo map hokuyo_link
```

Should print transform data without errors.

### 3. Visualize in RViz
```bash
rviz
```
- Add TF display
- Set Fixed Frame to `map`
- You should see all links moving as pan/tilt moves

## Updated Launch Sequence

```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Start TF publishers
roslaunch hokuyo_go robot_tf.launch

# Terminal 3: Start rosserial (Arduino)
rosrun rosserial_python serial_node.py /dev/ttyACM0

# Terminal 4: Start Hokuyo
rosrun urg_node urg_node

# Terminal 5: Start laser scanner
rosrun hokuyo_go laser_real_position

# Terminal 6: Visualize
rviz
```

## Troubleshooting

### "Transform timeout" errors
- Increase timeout: `tf_buffer->transform(..., ros::Duration(0.5))`
- Check TF is being published: `rostopic echo /tf`
- Verify timestamps match

### "Frame does not exist" errors
- Check frame names match exactly
- Run `rosrun tf view_frames` to see what frames exist
- Common issue: `hokuyo_link` vs `laser_link`

### Points appear in wrong location
- Check transform directions (parent/child)
- Verify rotation axes (pan=Z, tilt=Y)
- Check sign conventions (+ vs - rotations)

### Empty pointcloud
- Verify laser data: `rostopic echo /scan`
- Check frame_id in laser messages
- Ensure TF tree is complete

## Benefits of TF Approach

✅ **No hardcoded math** - kinematics handled by ROS
✅ **Easier to modify** - change URDF, not code
✅ **Standard ROS practice** - works with all ROS tools
✅ **Simulation compatibility** - same code works in Gazebo
✅ **Better debugging** - visualize transforms in RViz
✅ **Voxelization** - cleaner, more efficient pointclouds

## Next Steps

1. Choose one of the TF publishing methods above
2. Test TF tree with `rosrun tf view_frames`
3. Update Arduino topic names if needed (check frame_id in joint_states)
4. Run a test scan and verify pointcloud quality
5. Compare with old implementation results
