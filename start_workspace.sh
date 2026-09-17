#!/bin/bash
# start_workspace.sh - A single script to configure the system, start docker, and run source commands

# Ensure we are in the workspace root
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$WORKSPACE_DIR"

echo "=========================================="
echo "    Running System Configuration..."
echo "=========================================="
if systemctl is-active --quiet rccar-host-setup.service; then
    # Ran at boot (scripts/install_host_setup.sh), so no sudo here.
    echo "Host setup already done at boot by rccar-host-setup.service."
    if ! cmp -s ./configure_system.sh /usr/local/sbin/rccar-host-setup; then
        echo "Warning: configure_system.sh differs from the installed copy;"
        echo "         run 'sudo scripts/install_host_setup.sh' to apply the change."
    fi
elif [ -f "./configure_system.sh" ]; then
    ./configure_system.sh
else
    echo "Warning: ./configure_system.sh not found. Skipping."
fi

# docker/setup_bashrc.user.sh is mounted into the container's entrypoint_additions
# directory, where workspace-entrypoint.sh runs it as the 'admin' user on every
# container start. It is tracked once, under docker/, and never regenerated here.
SETUP_BASHRC="$WORKSPACE_DIR/docker/setup_bashrc.user.sh"

# run_dev.sh reads ~/.isaac_ros_dev-dockerargs (falling back to a copy next to
# itself, never to docker/) and ~/.isaac_ros_common-config, so keep both home
# files symlinks to the repo copies.
link_home_file() {
    local home_file="$HOME/$1"
    local repo_file
    repo_file="$(readlink -f "$WORKSPACE_DIR/docker/$1")"
    if [ "$(readlink -f "$home_file" 2>/dev/null)" != "$repo_file" ]; then
        if [ -e "$home_file" ] || [ -L "$home_file" ]; then
            mv --backup=numbered "$home_file" "$home_file.bak"
        fi
        if ln -s "$repo_file" "$home_file"; then
            echo "Linked $home_file -> $repo_file"
        else
            echo "Warning: could not link $home_file to $repo_file; run_dev.sh may use a stale copy"
        fi
    fi
}
link_home_file .isaac_ros_dev-dockerargs
link_home_file .isaac_ros_common-config

echo "=========================================="
echo "    Launching Isaac ROS Docker..."
echo "=========================================="

cd src/isaac_ros_common/scripts
# We pass docker arguments to mount our custom bashrc injection script so it runs at container startup
./run_dev.sh -d "$WORKSPACE_DIR" -a "-v ${SETUP_BASHRC}:/usr/local/bin/scripts/entrypoint_additions/setup_bashrc.user.sh"
