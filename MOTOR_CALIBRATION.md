# Automatic Motor Deadband Calibration

## Overview
The pan-tilt system now features automatic deadband calibration that runs on startup. This eliminates the need for manual tuning and makes the system portable across different motors and conditions.

## How It Works

### Calibration Process (runs automatically on power-up):
1. **Wait for ROS Connection** - Ensures ROS is ready before calibration
2. **Assume Home Position** - System assumes it starts at home (0, 0)
3. **Calibrate Both Motors Simultaneously**:
   - Gradually increases PWM from 0 to max (150) for each motor
   - Monitors encoders for movement (≥0.5 degrees)
   - Records minimum PWM that causes movement for each axis
4. **Initialize PID** - Configures PID with calibrated deadband values
5. **Return to Home Using PID** - Sets targets to (0, 0) and uses normal PID loop to return
6. **Reset Home Position** - Sets current position as new home after return
7. **Start Normal Operation** - Begins publishing joint states and accepting commands

**Key Optimization**: Uses the same PID controller for returning home, eliminating redundant control code.

## Monitoring Calibration

### View Calibration Logs
```bash
# Watch the calibration process in real-time
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

You should see output like:
```
========================================
STARTING AUTOMATIC DEADBAND CALIBRATION
Assumption: System is at home position
========================================
Calibrating TILT and PAN motors...
Calibrating TILT motor deadband...
TILT deadband found at PWM = 46 (moved 0.65 deg)
Calibrating PAN motor deadband...
PAN deadband found at PWM = 34 (moved 0.72 deg)
========================================
CALIBRATION COMPLETE!
Tilt deadband: 46 PWM
Pan deadband: 34 PWM
Now returning to home using PID...
========================================
Returning to home position...
Home position reached!
========================================
System ready for operation!
========================================
```

### View Calibration Results
```bash
# Check the calibrated deadband values (published every 5 seconds)
rostopic echo /motor_calibration
```

Example output:
```yaml
header:
  seq: 10
  stamp:
    secs: 1234567
    nsecs: 0
  frame_id: "calibration"
name: ['tilt_deadband', 'pan_deadband']
position: [46.0, 34.0]   # PWM values
velocity: []
effort: []
```

### Using rqt for Monitoring
```bash
# Launch rqt and navigate to Plugins → Topics → Topic Monitor
rqt

# Add /motor_calibration to monitor deadband values in real-time
```

## Calibration Parameters

You can adjust these in the Arduino code if needed:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `MAX_DEADBAND_SEARCH` | 150 | Maximum PWM to search |
| `DEADBAND_INCREMENT` | 2 | PWM step size |
| `ENCODER_MOVEMENT_THRESHOLD` | 0.5° | Minimum movement to detect |
| `DEADBAND_TEST_DURATION` | 500ms | How long to test each PWM level |

## Benefits

1. **Portability** - Works with different motors without code changes
2. **Adaptability** - Adjusts to wear, temperature, voltage variations
3. **Consistency** - Ensures optimal performance after every power cycle
4. **Efficiency** - Calibrates both motors simultaneously, uses existing PID for return-to-home
5. **Documentation** - Published deadband values help with debugging (every 5 seconds)
6. **Simple Operation** - Just power on with system at home position

## Important: Starting Position

⚠️ **The system assumes it starts at the home (center) position on power-up.**

- Position the pan-tilt mechanism at center before powering on
- Encoders will be zeroed at startup position
- After calibration, system returns to this position using PID

## Troubleshooting

### Motor doesn't move during calibration
- Check motor connections
- Verify power supply voltage
- Increase `MAX_DEADBAND_SEARCH` if needed

### Calibration takes too long
- Increase `DEADBAND_INCREMENT` (less precise but faster)
- Decrease `DEADBAND_TEST_DURATION` (may miss slow starts)

### False movement detection
- Increase `ENCODER_MOVEMENT_THRESHOLD`
- Check for mechanical vibrations

## Integration with ROS System

The calibrated deadband values are automatically used by:
- PID controller output limits
- Motor control deadband compensation
- Performance monitoring and logging

**Publishing Rates:**
- Joint states: 10 Hz (every 100ms)
- Calibration data: 0.2 Hz (every 5 seconds) - reduces ROS traffic

Total startup time: ~8-12 seconds (including calibration and return-to-home)
