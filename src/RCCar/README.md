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
  config/velocity_mux.yaml             velocity mux sources, priorities, timeouts
  config/my_custom_nav2_params.yaml    Nav2
  config/disable_shm.xml               FastDDS profile used inside the container
  behavior_trees/navigate_to_pose_car_like.xml   Recovery: reverse, pause, replan; three then abort
  launch/rccarauto.launch.py           the full stack
  launch/perception.launch.py          Perception bring-up: camera, visual SLAM, nvblox
  launch/perception_only.launch.py     Perception bring-up alone, for the camera bench
  test/test_perception_launch.py       the Perception bring-up interface, under colcon test
  test/test_vehicle_geometry.py        every value derived from the xacro's Vehicle geometry
  test/test_behavior_tree.py           the car-like tree's shape and limits
  scripts/                             see below
```

## Hardware interface

The ros2_control block in the URDF is the plugin's whole interface. Every parameter there is read and the plugin warns about any it does not recognise. The ESC calibration (offsets, output cap, dead-band, pulse widths, arming and direction-change dwells) lives only there.

- `steering_joint`: position command in radians. 0 maps to `neutral_pulse_us`; `min_angle` and `max_angle` map to `min_pulse_us` and `max_pulse_us`.
- `traction_joint`: velocity command in rear-wheel rad/s from the Steering controller. `max_wheel_speed_rad_s` maps that linearly onto the ESC output band between `forward_offset` and `max_output`.

There is no feedback. The state interfaces echo the command and `steer_bot_hardware.yaml` sets `open_loop: true`.

## Vehicle geometry and the costmaps

The measured dimensions live once, as xacro properties at the top of the URDF. The Steering controller's wheelbase and wheel radii, the Nav2 footprint (chassis box plus 2 cm), the planner's minimum turning radius floor and the Arrival tolerance (one car length) are derived from them, the nvblox obstacle band is checked against the ground clearance, and `test_vehicle_geometry.py` fails if any copy drifts.

Both costmaps read one obstacle source, nvblox's 2D occupancy grid on `/nvblox_node/static_occupancy_grid`, plus inflation. nvblox slices that grid from 5 cm voxel rows between the band edges in `vehicle_geometry.py`: the lower edge of 0.06 m picks the first row clear of the floor, so anything taller than about 2.5 cm registers and the floor does not. The band is fixed in the odom frame, whose z is zero where visual SLAM started, so it is a height above the floor on level ground and tilts with odom on a slope. Nav2 reads odometry from visual SLAM on `/visual_slam/tracking/odometry`. Everything is in the odom frame (ADR-0001).

## Build and run

Inside the Isaac ROS container:

```bash
colcon build --packages-select pca9685_hardware_interface rc_hardware_control --symlink-install
source install/setup.bash
ros2 launch rc_hardware_control rccarauto.launch.py
```

For camera work with the car still, `perception_only.launch.py` starts only the URDF and the Perception bring-up. Its arguments are `camera_profile` (848x480x30), `obstacle_band_lower_edge` in metres above the robot frame, `robot_frame` (base_footprint), and `camera_reset` (false; the Perception watchdog sets it after a start that never came up); they can be given on either launch's command line.

## Planning, Arrival, Stuck and Recovery

The planner is Smac Hybrid with the DUBIN motion model, so every plan is forward only. Arrival is being within one car length of the Goal with heading ignored; the tolerance latches once met. The car is Stuck when it has not moved 10 cm in 15 s. Path-following failure, Stuck included, runs a Recovery from `behavior_trees/navigate_to_pose_car_like.xml`: settle for a second, reverse 25 cm at 0.10 m/s, pause two seconds, and carry on with the plan the tree has been recomputing at 1 Hz all along. Three Recoveries, then the Goal aborts. Planning failure aborts at once, so a Goal in a wall never moves the car. There is no spin, no drive-on-heading and no assisted teleop. `rccarauto.launch.py` rewrites the Nav2 params file so the behaviour tree parameter points at the installed tree.

## Velocity mux and teleop

The Steering controller (`bicycle_steering_controller`) listens only to `/cmd_vel_mux`, the velocity mux's TwistStamped output. The mux has two sources, teleop on `/cmd_vel_teleop` and Nav2 on `/cmd_vel`, and teleop always wins. A source that goes quiet for longer than its timeout drops out (teleop 1.0 s, Nav2 0.5 s), so releasing teleop hands the car back to Nav2 if a Goal is active, and if nothing is fresh the mux publishes zero and the car stops. The values and the reasoning are in `config/velocity_mux.yaml`.

Teleop is the Foxglove Teleop panel publishing `geometry_msgs/Twist` on `/cmd_vel_teleop`; its 5 Hz default works, 10 Hz tolerates more Wi-Fi loss. Over ssh, the keyboard fallback is:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_teleop
```

To drive the car from a script on the bench, publish to the teleop topic so the mux passes it through:

```bash
ros2 run rc_hardware_control test_bicycle.py     # scripted forward, turns, reverse, stop
```

Tests run without hardware: gtests for the mapping code, pytest for the perception launch interface.

```bash
colcon test --packages-select pca9685_hardware_interface rc_hardware_control
```

## Scripts

| Script | Purpose |
| --- | --- |
| `test_bicycle.py` | Publishes a fixed sequence of Twist commands on `/cmd_vel_teleop` to exercise steering and traction. |
| `cmd_vel_logger.py` | Prints every `/cmd_vel` message, for watching what Nav2 sends. Remap it to watch another topic. |
| `velocity_mux.py` | The velocity mux: Twist in from teleop and Nav2, TwistStamped out. Started by the launch with `config/velocity_mux.yaml`. |
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
