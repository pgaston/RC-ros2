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

# docker/setup_bashrc.user.sh is mounted into the container's entrypoint_additions
# directory, where workspace-entrypoint.sh runs it as the 'admin' user on every
# container start. It is tracked once, under docker/, and never regenerated here.
SETUP_BASHRC="$WORKSPACE_DIR/docker/setup_bashrc.user.sh"

# run_dev.sh reads ~/.isaac_ros_dev-dockerargs (falling back to a copy next to
# itself, never to docker/), so keep the home file a symlink to the repo copy.
ARGS_FILE="$HOME/.isaac_ros_dev-dockerargs"
REPO_ARGS_FILE="$(readlink -f "$WORKSPACE_DIR/docker/.isaac_ros_dev-dockerargs")"
if [ "$(readlink -f "$ARGS_FILE" 2>/dev/null)" != "$REPO_ARGS_FILE" ]; then
    if [ -e "$ARGS_FILE" ] || [ -L "$ARGS_FILE" ]; then
        mv --backup=numbered "$ARGS_FILE" "$ARGS_FILE.bak"
    fi
    if ln -s "$REPO_ARGS_FILE" "$ARGS_FILE"; then
        echo "Linked $ARGS_FILE -> $REPO_ARGS_FILE"
    else
        echo "Warning: could not link $ARGS_FILE to $REPO_ARGS_FILE; run_dev.sh may use stale docker args"
    fi
fi

echo "=========================================="
echo "    Launching Isaac ROS Docker..."
echo "=========================================="

cd src/isaac_ros_common/scripts
# We pass docker arguments to mount our custom bashrc injection script so it runs at container startup
./run_dev.sh -d "$WORKSPACE_DIR" -a "-v ${SETUP_BASHRC}:/usr/local/bin/scripts/entrypoint_additions/setup_bashrc.user.sh"
