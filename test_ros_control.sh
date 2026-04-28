#!/bin/bash
source /workspaces/isaac_ros-dev/install/setup.bash
ros2 launch rc_hardware_control rccarauto.launch.py > launch_output.log 2>&1 &
LAUNCH_PID=$!
sleep 15
ros2 control list_controllers
kill $LAUNCH_PID
sleep 2
