# PCA9685 ROS2 Control with Steering and Tracking Joints

This updated PCA9685 hardware interface supports both **steering** (position control) and **tracking** (velocity control) joints for robotic applications.  
  
This is based **specifically** on an [existing repository](https://github.com/rosblox/pca9685_ros2_control) - thanks! - and modified to:
- add more features so as to fully support what I need for an RC car
- a custom ros2 node for my car - see rc_hardware_control

Note that I removed the pan/tilt for now.   AI can add back easily...   :)

## Features

- **Position Control**: For servos (steering, pan/tilt cameras)
- **Velocity Control**: For ESCs (traction motors, continuous rotation)
- **Mixed Joint Types**: Support both position and velocity joints in the same robot
- **Configurable Parameters**: Channel mapping, pulse width limits, angle ranges

## Hardware Configuration

### Joint Types

1. **Steering Joints** (Position Control)
   - Used for: Servo steering, camera pan/tilt
   - Interface: `position`
   - Control: Angle in radians → PWM pulse width
   - Parameters: `min_angle`, `max_angle`, `min_pulse_us`, `max_pulse_us`

2. **Tracking Joints** (Velocity Control)  
   - Used for: ESC motor control, continuous rotation
   - Interface: `velocity`
   - Control: Velocity command → PWM duty cycle
   - Parameters: Only `channel` required

### URDF Configuration

```xml
<joint name="steering_joint" type="revolute">
  <command_interface name="position" />
  <state_interface name="position" />
  <param name="channel">0</param>
  <param name="min_angle">-90</param>
  <param name="max_angle">90</param>
  <param name="min_pulse_us">1000</param>
  <param name="max_pulse_us">2000</param>
</joint>

<joint name="traction_joint" type="continuous">
  <command_interface name="velocity" />
  <state_interface name="velocity" />
  <param name="channel">1</param>
</joint>
```

### Controller Configuration

```yaml
# Position controller for steering
steering_controller:
  ros__parameters:
    joints:
      - steering_joint

# Velocity controller for tracking
tracking_controller:
  ros__parameters:
    joints:
      - traction_joint
```

## Usage

### 1. Launch the System

```bash
# Launch steering and tracking example
ros2 launch pca9685_ros2_control_example steering_tracking_example.launch.py

# Or launch joint group velocity example
ros2 launch pca9685_ros2_control_example joint_group_velocity_example.launch.py
```

### 2. Control Commands

#### Ackermann Steering Control (Recommended)
```bash
# Forward motion
ros2 topic pub /ackermann_steering_controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.0}}' --once

# Turn right while moving forward
ros2 topic pub /ackermann_steering_controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: -0.5}}' --once

# Turn left while moving forward  
ros2 topic pub /ackermann_steering_controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.5}}' --once

# Spin in place (point turn)
ros2 topic pub /ackermann_steering_controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 1.0}}' --once

# Stop
ros2 topic pub /ackermann_steering_controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' --once
```

#### LED Brightness Control
```bash
# Full brightness
ros2 topic pub /led_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0]" --once

# Half brightness
ros2 topic pub /led_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]" --once

# LED off
ros2 topic pub /led_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]" --once
```

### 3. Test Scripts

Run the automated test scripts:
```bash
# Test Ackermann steering (recommended)
ros2 run pca9685_ros2_control_example test_ackermann.py

# Test individual joint control (legacy)
ros2 run pca9685_ros2_control_example test_steering_tracking.py
```

## Channel Mapping

The example configuration uses:
- **Channel 0**: Steering servo (position)
- **Channel 1**: Traction ESC (velocity)  
- **Channel 2**: Pan servo (position)
- **Channel 3**: Tilt servo (position)
- **Channel 15**: LED brightness (effort/PWM)

## PWM Signal Details

### Position Control (Servos)
- **Frequency**: 50Hz (20ms period)
- **Pulse Width**: 1000-2000µs typically
- **Angle Mapping**: Linear interpolation between min/max angles and pulse widths

### Velocity Control (ESCs)
- **Frequency**: 50Hz (20ms period)
- **Pulse Width**: 
  - 1000µs = Full reverse
  - 1500µs = Stop/neutral
  - 2000µs = Full forward

### LED PWM Control
- **Frequency**: 50Hz (20ms period)
- **Duty Cycle**: 0-100% brightness
- **Command Range**: 0.0 (off) to 1.0 (full brightness)

## Troubleshooting

### Build Issues
```bash
# Install dependencies
sudo apt install libi2c-dev

# Build the package
cd /workspaces/isaac_ros-dev
colcon build --packages-select pca9685_hardware_interface pca9685_ros2_control_example

# Source the workspace
source install/setup.bash
```

### Hardware Issues
1. **Check I2C permissions**: `sudo chmod 666 /dev/i2c-*`
2. **Verify PCA9685 address**: Default is 0x40 (decimal 64)
3. **Check connections**: SDA, SCL, VCC, GND to PCA9685
4. **Power supply**: Ensure adequate power for servos/ESCs

### Controller Issues
```bash
# List available controllers
ros2 control list_controllers

# Check joint states
ros2 topic echo /joint_states

# Monitor controller manager
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

## Advanced Configuration

### Custom Servo Ranges
For 180-degree servos with 500-2500µs range:
```xml
<param name="min_angle">-90</param>
<param name="max_angle">90</param>
<param name="min_pulse_us">500</param>
<param name="max_pulse_us">2500</param>
```

### Multiple PCA9685 Boards
Add multiple hardware interfaces with different I2C addresses:
```xml
<hardware>
  <plugin>pca9685_hardware_interface/Pca9685SystemHardware</plugin>
  <param name="address">64</param>  <!-- 0x40 -->
</hardware>

<hardware>
  <plugin>pca9685_hardware_interface/Pca9685SystemHardware</plugin>  
  <param name="address">65</param>  <!-- 0x41 -->
</hardware>
```