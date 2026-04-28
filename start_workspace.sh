#!/bin/bash
# start_workspace.sh - A single script to configure the system, start docker, and run source commands

# Ensure we are in the workspace root
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$WORKSPACE_DIR"

echo "=========================================="
echo "    Running System Configuration..."
echo "=========================================="
if [ -f "./configure_system.sh" ]; then
    sudo ./configure_system.sh
else
    echo "Warning: ./configure_system.sh not found. Skipping."
fi

# We create an entrypoint script that appends our launch commands to ~/.bashrc inside Docker
# This will be automatically executed as the 'admin' user by workspace-entrypoint.sh
SCRIPT_NAME=".docker_setup_bashrc.user.sh"

cat << 'EOF' > "$SCRIPT_NAME"
#!/bin/bash
BASHRC="$HOME/.bashrc"
if ! grep -q "# Auto-injected ROS Dev Setup" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# Auto-injected ROS Dev Setup" >> "$BASHRC"
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/src/RCCar/rc_hardware_control/config/disable_shm.xml" >> "$BASHRC"
    echo "cd /workspaces/isaac_ros-dev" >> "$BASHRC"

    echo "if [ -f /workspaces/isaac_ros-dev/source_dev.sh ]; then" >> "$BASHRC"
    echo "    source /workspaces/isaac_ros-dev/source_dev.sh" >> "$BASHRC"
    echo "fi" >> "$BASHRC"
    echo "if [ -f /workspaces/isaac_ros-dev/install/setup.bash ]; then" >> "$BASHRC"
    echo "    source /workspaces/isaac_ros-dev/install/setup.bash" >> "$BASHRC"
    echo "fi" >> "$BASHRC"
fi
EOF
chmod +x "$SCRIPT_NAME"

echo "=========================================="
echo "    Launching Isaac ROS Docker..."
echo "=========================================="

cd src/isaac_ros_common/scripts
# We pass docker arguments to mount our custom bashrc injection script so it runs at container startup
./run_dev.sh -d "$WORKSPACE_DIR" -a "-v ${WORKSPACE_DIR}/${SCRIPT_NAME}:/usr/local/bin/scripts/entrypoint_additions/setup_bashrc.user.sh"
