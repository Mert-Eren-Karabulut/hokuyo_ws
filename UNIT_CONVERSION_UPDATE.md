# Unit Conversion Update Summary

## Problem
The Arduino code was publishing joint positions in **degrees**, but ROS standard dictates that `sensor_msgs/JointState` should use **radians**. This would cause `robot_state_publisher` to interpret the transforms incorrectly.

## Solution Applied

### Arduino Code Changes (`servo_ros.ino`)

**Added conversion constants:**
```cpp
const float DEG_TO_RAD = PI / 180.0;
const float RAD_TO_DEG = 180.0 / PI;
```

**Updated `jointCommandCallback` to receive radians:**
```cpp
// ROS sends radians → Convert to degrees for internal PID
float tiltTarget = msg.position[0] * RAD_TO_DEG;
float panTarget = msg.position[1] * RAD_TO_DEG;
```

**Updated `updateJointStateData` to publish radians:**
```cpp
// Internal degrees → Convert to radians for ROS
joint_values[0] = tiltInput * DEG_TO_RAD;   // Tilt position
joint_values[1] = panInput * DEG_TO_RAD;    // Pan position
joint_values[4] = (tiltSetpoint - tiltInput) * DEG_TO_RAD;  // Tilt error
joint_values[5] = (panSetpoint - panInput) * DEG_TO_RAD;    // Pan error
joint_values[6] = tiltSetpoint * DEG_TO_RAD;  // Tilt setpoint
joint_values[7] = panSetpoint * DEG_TO_RAD;   // Pan setpoint
```

### C++ Code Updates (`laser_real_position.cpp`)

**Updated comments for clarity:**
- Made it explicit that positions are in radians throughout
- No computational changes needed (code already expected radians)

### URDF Updates (`real_pantilt.urdf`)

**Changed joint names to match Arduino:**
- `pan_joint` → `joint2` (matches Arduino's `joint_names[1]`)
- `tilt_joint` → `joint1` (matches Arduino's `joint_names[0]`)

## Internal vs External Units

### Arduino Internal (Private)
- **Encoders:** 4000 counts/revolution
- **Conversion:** `DEGREES_PER_COUNT = 360.0 / 4000.0 = 0.09°`
- **PID Controller:** Works in **degrees**
- **Setpoints:** Stored in **degrees**

### ROS Communication (Public Interface)
- **`/joint_states` (published):** Positions in **radians**
- **`/joint_command` (subscribed):** Positions in **radians**
- **TF Transforms:** All angles in **radians**

## Data Flow

```
┌──────────────────────────────────────┐
│         Arduino Controller           │
├──────────────────────────────────────┤
│  Encoder → degrees (internal)        │
│  PID Controller → degrees            │
│  ↓                                   │
│  Convert to radians (DEG_TO_RAD)     │
│  ↓                                   │
│  Publish /joint_states (radians)     │
└──────────────────┬───────────────────┘
                   ↓
         /joint_states (radians)
                   ↓
┌──────────────────┴───────────────────┐
│      robot_state_publisher           │
│  Reads URDF + /joint_states          │
│  Publishes TF (all angles radians)   │
└──────────────────┬───────────────────┘
                   ↓
         TF Transform Tree (radians)
                   ↓
┌──────────────────┴───────────────────┐
│    laser_real_position.cpp           │
│  Uses TF to transform laser points   │
│  Publishes /joint_command (radians)  │
└──────────────────┬───────────────────┘
                   ↓
         /joint_command (radians)
                   ↓
┌──────────────────┴───────────────────┐
│         Arduino Controller           │
│  ↓                                   │
│  Convert to degrees (RAD_TO_DEG)     │
│  ↓                                   │
│  PID Controller → degrees            │
│  Motor Control → PWM                 │
└──────────────────────────────────────┘
```

## Verification Steps

### 1. Check Joint State Messages
```bash
rostopic echo /joint_states
```
Expected output (when at 45°):
```yaml
position: [0.785398, 0.0, ...]  # 45° = 0.785398 rad
```

### 2. Verify TF Transforms
```bash
rosrun tf tf_echo base_link hokuyo_link
```
Should show reasonable rotation angles in radians.

### 3. Test Joint Commands
```bash
# Command 45 degrees tilt (0.785398 radians)
rostopic pub /joint_command sensor_msgs/JointState "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
name: ['joint1', 'joint2']
position: [0.785398, 0.0]
velocity: []
effort: []"
```

Arduino should move tilt motor to 45°.

## Files Modified

1. **`/home/isl9/dev/mert/arduino_codes/servo_ros/servo_ros.ino`**
   - Added `DEG_TO_RAD` and `RAD_TO_DEG` constants
   - Modified `jointCommandCallback()`: Convert incoming radians to degrees
   - Modified `updateJointStateData()`: Convert outgoing degrees to radians

2. **`/home/isl9/dev/mert/hokuyo_ws/src/hokuyo_go/src/laser_real_position.cpp`**
   - Updated comments to clarify radians usage
   - No computational changes (already correct)

3. **`/home/isl9/dev/mert/hokuyo_ws/src/hokuyo_go/urdf/real_pantilt.urdf`**
   - Renamed `pan_joint` to `joint2`
   - Renamed `tilt_joint` to `joint1`
   - Matches Arduino's joint naming convention

## Next Steps

1. **Upload updated Arduino code** to the board
2. **Rebuild the ROS workspace:**
   ```bash
   cd /home/isl9/dev/mert/hokuyo_ws
   catkin_make
   ```
3. **Test the system:**
   ```bash
   # Terminal 1: roscore
   roscore
   
   # Terminal 2: Arduino serial bridge
   rosrun rosserial_python serial_node.py /dev/ttyACM0
   
   # Terminal 3: Launch the system
   roslaunch hokuyo_go real_pantilt.launch
   
   # Terminal 4: Verify TF
   rosrun tf view_frames
   rosrun tf tf_echo map hokuyo_link
   ```

## Benefits

✅ **ROS Standard Compliance:** All joint positions now use radians  
✅ **robot_state_publisher Compatible:** TF transforms will be correct  
✅ **Maintainable:** Clear separation between internal and external units  
✅ **Debuggable:** Easy to verify with standard ROS tools  
✅ **Interoperable:** Works with any ROS node expecting standard JointState messages
