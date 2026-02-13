#!/bin/bash

# Source ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source Workspace
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

# Apply LD_LIBRARY_PATH fix for Isaac ROS GXF Serialization
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization

echo "Environment sourced. LD_LIBRARY_PATH updated."
