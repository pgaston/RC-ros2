#!/bin/bash
BASHRC="$HOME/.bashrc"
if ! grep -q "# Auto-injected ROS Dev Setup" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# Auto-injected ROS Dev Setup" >> "$BASHRC"
    echo "cd /workspaces/isaac_ros-dev" >> "$BASHRC"
    echo "if [ -f /workspaces/isaac_ros-dev/source_dev.sh ]; then" >> "$BASHRC"
    echo "    source /workspaces/isaac_ros-dev/source_dev.sh" >> "$BASHRC"
    echo "fi" >> "$BASHRC"
    echo "if [ -f /workspaces/isaac_ros-dev/install/setup.bash ]; then" >> "$BASHRC"
    echo "    source /workspaces/isaac_ros-dev/install/setup.bash" >> "$BASHRC"
    echo "fi" >> "$BASHRC"
fi
