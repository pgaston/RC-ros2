# RCCar packages

Two ROS 2 Humble packages that make a hobby RC car drivable from ros2_control.

- **pca9685_hardware_interface**: the Hardware interface. A ros2_control system plugin that turns a Steering joint angle and a Traction joint wheel speed into servo and ESC pulses on a PCA9685 over I²C. It is based on rosblox's PCA9685 ros2_control hardware interface on GitHub, reworked for one servo and one ESC with no feedback.
- **rc_hardware_control**: the car itself. URDF, Steering controller configuration, Nav2 parameters, the full-stack launch, and a few helper scripts.

The vocabulary (Traction joint, Steering joint, Steering controller, Hardware interface, Goal relay, Perception bring-up, Vehicle geometry) is defined in `CONTEXT.md` at the repo root.

## Layout

```
pca9685_hardware_interface/
  include/, src/        plugin, I²C adapter, pure mapping code
  test/                 gtests for the servo and traction mapping (no I²C needed)
  TestPCA9685ESC.py     Adafruit bench scripts for calibrating the ESC and servo
  TestPCA9685ServoESC.py  without ROS; the numbers they find go into the URDF
rc_hardware_control/
  description/description.urdf.xacro   vehicle geometry and the ros2_control block
  config/steer_bot_hardware.yaml       controller manager and the Steering controller
  config/my_custom_nav2_params.yaml    Nav2
  config/disable_shm.xml               FastDDS profile used inside the container
  launch/rccarauto.launch.py           the full stack
  scripts/                             see below
```

## Hardware interface

The ros2_control block in the URDF is the plugin's whole interface. Every parameter there is read and the plugin warns about any it does not recognise. The ESC calibration (offsets, output cap, dead-band, pulse widths, arming and direction-change dwells) lives only there.

- `steering_joint`: position command in radians. 0 maps to `neutral_pulse_us`; `min_angle` and `max_angle` map to `min_pulse_us` and `max_pulse_us`.
- `traction_joint`: velocity command in rear-wheel rad/s from the Steering controller. `max_wheel_speed_rad_s` maps that linearly onto the ESC output band between `forward_offset` and `max_output`.

There is no feedback. The state interfaces echo the command and `steer_bot_hardware.yaml` sets `open_loop: true`.

## Build and run

Inside the Isaac ROS container:

```bash
colcon build --packages-select pca9685_hardware_interface rc_hardware_control --symlink-install
source install/setup.bash
ros2 launch rc_hardware_control rccarauto.launch.py
```

The launch remaps the Steering controller's (`bicycle_steering_controller`) reference topics onto `/cmd_vel`, so anything that publishes `geometry_msgs/Twist` there drives the car:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run rc_hardware_control test_bicycle.py     # scripted forward, turns, reverse, stop
```

Unit tests for the mapping code run without hardware:

```bash
colcon test --packages-select pca9685_hardware_interface
```

## Scripts

| Script | Purpose |
| --- | --- |
| `test_bicycle.py` | Publishes a fixed sequence of `/cmd_vel` commands to exercise steering and traction. |
| `cmd_vel_logger.py` | Prints every `/cmd_vel` message, for watching what Nav2 or teleop sends. |
| `frame_rename.py` | Republishes the infra2 camera info with the frame id visual SLAM expects. Started by the launch. |
| `goal_pose_relay.py` | The Goal relay. Forwards `/goal_pose` and `/clicked_point` to Nav2's NavigateToPose action. |
| `rs-imu-calibration.py` | Intel's RealSense IMU calibration tool, kept for bench use. Not a ROS node. |

## Troubleshooting

```bash
sudo chmod 666 /dev/i2c-*          # I²C permission inside the container
ros2 control list_controllers      # bicycle_steering_controller (the Steering controller) should be active
ros2 topic echo /joint_states      # echoes the commanded angle and speed
```

The PCA9685 is expected at address 64 (0x40) on `/dev/i2c-7`; both are parameters in the URDF hardware block.
