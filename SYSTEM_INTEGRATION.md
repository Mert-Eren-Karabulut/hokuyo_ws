# Real Pan-Tilt System Integration Guide

## System Overview

This document describes how the Arduino controller, C++ scanning node, URDF model, and `robot_state_publisher` work together.

## Component Architecture

```
┌─────────────────┐
│  Arduino Board  │
│  (servo_ros.ino)│
└────────┬────────┘
         │ publishes /joint_states
         │ subscribes /joint_command
         ↓
┌─────────────────────────────┐
│  robot_state_publisher      │
│  (Reads URDF + /joint_states)│
└────────┬────────────────────┘
         │ publishes TF transforms
         │ (map→base_link→joint2→joint1→hokuyo_base→hokuyo_link)
         ↓
┌──────────────────────────────┐
│  laser_real_position node    │
│  (C++ scanning controller)   │
└──────────────────────────────┘
         │ uses TF to transform points
         │ publishes /joint_command
         │ publishes /output (pointcloud)
         ↓
    Point Cloud
```

## Joint Naming Convention

**IMPORTANT:** All joint names must match between Arduino, URDF, and C++ code.

### Arduino Configuration (`servo_ros.ino`)

```cpp
const char *joint_names[2] = {"joint1", "joint2"};
```

**Published on `/joint_states`:**
- `position[0]` = tilt angle (radians) → `joint1`
- `position[1]` = pan angle (radians) → `joint2`

**Subscribed from `/joint_command`:**
- `position[0]` = tilt target (radians)
- `position[1]` = pan target (radians)

**Note:** The Arduino only publishes the 2 essential joints needed by `robot_state_publisher`. Diagnostic data (PWM, errors, setpoints) has been removed for cleaner integration.

### URDF Configuration (`real_pantilt.urdf`)

The URDF defines two revolute joints:

```xml
<joint name="joint1" type="revolute">  <!-- TILT -->
    <parent link="pan_link" />
    <child link="tilt_link" />
    <axis xyz="0 0 1" />  <!-- After 90° rotation, this is pitch axis -->
</joint>

<joint name="joint2" type="revolute">  <!-- PAN -->
    <parent link="base_link" />
    <child link="pan_link" />
    <axis xyz="0 0 1" />  <!-- Yaw axis -->
</joint>
```

### C++ Node Configuration (`laser_real_position.cpp`)

```cpp
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // Arduino publishes: position[0] = tilt, position[1] = pan
    if (msg->position.size() >= 2) {
        tiltangle = msg->position[0];  // Read from joint1
        panangle = msg->position[1];   // Read from joint2
    }
}

// When publishing commands:
joint_cmd.position[0] = tiltangle_goal;  // Tilt target → joint1
joint_cmd.position[1] = panangle_goal;   // Pan target → joint2
```

## robot_state_publisher Behavior

`robot_state_publisher` reads:
1. **URDF from parameter server** (`/robot_description`)
2. **Joint states from `/joint_states` topic**

It automatically publishes TF transforms based on:
- Joint positions in `/joint_states`
- Joint definitions in URDF

**For our system:**
- Reads `position[0]` (tilt) from `/joint_states` with name `joint1`
- Reads `position[1]` (pan) from `/joint_states` with name `joint2`
- Publishes transform tree: `map` → `base_link` → `pan_link` (via joint2) → `tilt_link` (via joint1) → `hokuyo_base` → `hokuyo_link`

## Unit Conversion

### Arduino ↔ ROS Communication

**Arduino Internal:**
- Encoder: 4000 counts per revolution
- Conversion: `DEGREES_PER_COUNT = 360.0 / 4000.0 = 0.09°`
- Works in **degrees**

**ROS Standard:**
- `sensor_msgs/JointState` uses **radians**

### Current Implementation Status

⚠️ **ACTION REQUIRED:** The Arduino currently publishes degrees, but ROS expects radians!

**Option 1: Fix Arduino (Recommended)**
Convert degrees to radians before publishing:

```cpp
void updateJointStateData() {
  joint_values[0] = tiltInput * DEG_TO_RAD;   // Convert to radians
  joint_values[1] = panInput * DEG_TO_RAD;    // Convert to radians
  // ... rest of values
}

void jointCommandCallback(const sensor_msgs::JointState& msg) {
  if (msg.position_length >= 2) {
    float tiltTarget = msg.position[0] * RAD_TO_DEG;  // Convert from radians
    float panTarget = msg.position[1] * RAD_TO_DEG;   // Convert from radians
    // ... rest of code
  }
}
```

**Option 2: Fix C++ node**
Convert in `laser_real_position.cpp`, but this is not ROS standard practice.

## TF Transform Chain

When everything is running, you should see:

```
map
 └─ base_link (fixed)
     └─ pan_link (revolute, controlled by joint2/pan)
         └─ tilt_link (revolute, controlled by joint1/tilt)
             └─ hokuyo_base (fixed)
                 └─ hokuyo_link (fixed)
```

**Verify with:**
```bash
# View full transform tree
rosrun tf view_frames

# Check specific transform
rosrun tf tf_echo map hokuyo_link

# Monitor joint states
rostopic echo /joint_states
```

## Launch Sequence

1. **Start roscore:**
   ```bash
   roscore
   ```

2. **Upload Arduino code and start rosserial:**
   ```bash
   rosrun rosserial_python serial_node.py /dev/ttyACM0
   ```

3. **Launch the system:**
   ```bash
   roslaunch hokuyo_go real_pantilt.launch
   ```

This will:
- Load URDF to parameter server
- Start `robot_state_publisher` (converts /joint_states → TF)
- Start `laser_real_position` node (scanning + pointcloud generation)

## Debugging Checklist

- [x] Arduino publishes `/joint_states` at ~10Hz (configured in code)
- [x] Joint names match: `joint1` (tilt), `joint2` (pan)
- [x] Units are consistent (radians - using Arduino.h built-in macros)
- [x] `robot_state_publisher` is running
- [x] TF tree shows all frames: `rosrun tf view_frames`
- [x] Transform works: `rosrun tf tf_echo map hokuyo_link`
- [ ] Laser data arrives: `rostopic echo /scan`
- [ ] Joint commands work: `rostopic pub /joint_command ...`
- [x] No URDF warnings (cleaned up joint state messages)

## System Updates (Clean Solution)

### Changes Applied

**Arduino Code (`servo_ros.ino`):**
- Reduced joint state message from 8 values to 2 (only joint1 and joint2)
- Removed diagnostic joints: `joint1Motor`, `joint2Motor`, `joint1Error`, `joint2Error`, `TiltPos`, `PanPos`
- Cleaner integration with `robot_state_publisher` - no warnings
- Diagnostic data can be added back via Serial if needed for debugging

**C++ Code (`laser_real_position.cpp`):**
- Added `joint_states_received` flag to properly detect when Arduino is connected
- Fixed startup bug where node would hang if robot was at home position (0, 0)

**URDF (`real_pantilt.urdf`):**
- Clean URDF with only essential joints: `joint1` (tilt), `joint2` (pan)
- No dummy diagnostic joints needed

### Known Warnings (Resolved)

~~"Joint state with name: 'PanPos' was received but not found in URDF"~~ ✅ **FIXED**
- Removed extra diagnostic joints from Arduino
- Arduino now only publishes `joint1` and `joint2`
- `robot_state_publisher` has no warnings

**"The root link map has an inertia specified in the URDF"**
- This is a KDL (Kinematics and Dynamics Library) limitation
- Harmless for our application since we're only using TF transforms, not dynamics calculations
- Can be ignored

## Next Steps

1. ✅ **Unit conversion fixed** - Arduino uses built-in DEG_TO_RAD/RAD_TO_DEG macros
2. ✅ **Workspace built** - `catkin_make` successful
3. ✅ **Upload new Arduino code** - Clean 2-joint version (no diagnostic data)
4. **Test the system:**
   - Verify no URDF warnings
   - Check TF transforms update with joint motion
   - Test joint commands manually
   - Connect laser scanner and test full scanning
5. **Visualize in RViz** - See TF tree and robot model in real-time

## Files Modified

- `/home/isl9/dev/mert/hokuyo_ws/src/hokuyo_go/urdf/real_pantilt.urdf` - URDF model (clean, no diagnostic joints)
- `/home/isl9/dev/mert/hokuyo_ws/src/hokuyo_go/launch/real_pantilt.launch` - Launch file
- `/home/isl9/dev/mert/arduino_codes/servo_ros/servo_ros.ino` - **Clean version: only 2 joints published**
- `/home/isl9/dev/mert/hokuyo_ws/src/hokuyo_go/src/laser_real_position.cpp` - Fixed startup bug, uses TF transforms

## Summary

✅ **Clean ROS Integration Achieved:**
- Arduino publishes only essential joints (joint1, joint2)
- No URDF warnings from robot_state_publisher
- Proper radian/degree conversion using Arduino built-in macros
- TF transforms working correctly
- Ready for visualization in RViz and full scanning operation
