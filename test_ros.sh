#!/bin/bash
source /workspaces/isaac_ros-dev/install/setup.bash
echo "Starting launch file..."
ros2 launch rc_hardware_control rccarauto.launch.py > launch_output.log 2>&1 &
LAUNCH_PID=$!
sleep 15
echo "Nodes:"
ros2 node list
echo "Topics:"
ros2 topic list
echo "Bicycle Controller Params:"
ros2 param dump /bicycle_steering_controller
echo "Killing launch..."
kill $LAUNCH_PID
sleep 2
