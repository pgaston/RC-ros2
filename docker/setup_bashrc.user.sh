#!/bin/bash
BASHRC="$HOME/.bashrc"
touch "$BASHRC"   # a fresh container has none yet; the greps below need it

# Clean up stale FastDDS shared memory files. /dev is the host's (issue #9),
# so files left by a root process cannot be removed from here; that is fine,
# the container's FastDDS profile disables shared memory anyway. The profile
# holds only with ROS_LOCALHOST_ONLY=0, which docker/.isaac_ros_dev-dockerargs
# sets over the base image's 1; the profile keeps DDS on loopback (issue #17).
for f in /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*; do
    [ -e "$f" ] && [ -O "$f" ] && rm -f "$f"
done

if ! grep -q "export ROS_DOMAIN_ID=" "$BASHRC"; then
    echo "export ROS_DOMAIN_ID=47" >> "$BASHRC"
fi

if ! grep -q "export FASTRTPS_DEFAULT_PROFILES_FILE=" "$BASHRC"; then
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/src/RCCar/rc_hardware_control/config/disable_shm.xml" >> "$BASHRC"
fi

if ! grep -q "export ROS_LOCALHOST_ONLY=" "$BASHRC"; then
    echo "export ROS_LOCALHOST_ONLY=0" >> "$BASHRC"
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
