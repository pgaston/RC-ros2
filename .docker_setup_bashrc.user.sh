#!/bin/bash
BASHRC="$HOME/.bashrc"

# Clean up stale FastDDS shared memory files
rm -f /dev/shm/fastrtps_*
rm -f /dev/shm/sem.fastrtps_*

if ! grep -q "export ROS_DOMAIN_ID=" "$BASHRC"; then
    echo "export ROS_DOMAIN_ID=47" >> "$BASHRC"
fi

if ! grep -q "export FASTRTPS_DEFAULT_PROFILES_FILE=" "$BASHRC"; then
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/src/RCCar/rc_hardware_control/config/disable_shm.xml" >> "$BASHRC"
fi

if ! grep -q "export ROS_LOCALHOST_ONLY=" "$BASHRC"; then
    echo "export ROS_LOCALHOST_ONLY=1" >> "$BASHRC"
fi

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
