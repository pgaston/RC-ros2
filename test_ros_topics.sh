#!/bin/bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch rc_hardware_control rccarauto.launch.py > launch_output.log 2>&1 &
LAUNCH_PID=$!
sleep 15
echo "Topic Info cmd_vel:"
ros2 topic info /cmd_vel --verbose
echo "Topic Info bicycle_steering_controller/reference_unstamped:"
ros2 topic info /bicycle_steering_controller/reference_unstamped --verbose
kill $LAUNCH_PID
sleep 2
