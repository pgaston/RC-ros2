#!/bin/bash
# Checks that the Isaac container can follow a RealSense re-enumeration (issue #9).
#
# Static check (always): the docker run arguments file that run_dev.sh reads
# bind-mounts the host's /dev, so device nodes created after container start
# are visible inside it. run_dev.sh reads ~/.isaac_ros_dev-dockerargs, falling
# back to a copy next to itself under src/isaac_ros_common/scripts; it never
# reads docker/.isaac_ros_dev-dockerargs directly, so the home file must be a
# symlink to the repo copy (start_workspace.sh keeps it so).
#
# Live check (when a container name is given): the container's /dev/video*
# list matches the host's /sys/class/video4linux right now. Run it after
# `usbreset 8086:0b3a` on the host to prove the view is live, not a snapshot.
#
# Usage: scripts/check_container_devices.sh [container-name]
set -u

WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd -P )"
REPO_ARGS_FILE="$WORKSPACE_DIR/docker/.isaac_ros_dev-dockerargs"
HOME_ARGS_FILE="$HOME/.isaac_ros_dev-dockerargs"
fail=0

if [ "$(readlink -f "$HOME_ARGS_FILE" 2>/dev/null)" = "$REPO_ARGS_FILE" ] && [ -e "$HOME_ARGS_FILE" ]; then
    echo "ok   ~/.isaac_ros_dev-dockerargs points at the repo copy"
else
    echo "FAIL ~/.isaac_ros_dev-dockerargs is missing, dangling, or not a symlink to $REPO_ARGS_FILE; run_dev.sh will not read the repo copy (start_workspace.sh fixes this)"
    fail=1
fi

if grep -qE -- '^-v[[:space:]]+/dev:/dev([[:space:]]|$)' "$REPO_ARGS_FILE"; then
    echo "ok   $REPO_ARGS_FILE bind-mounts /dev"
else
    echo "FAIL $REPO_ARGS_FILE does not bind-mount /dev (expected a line: -v /dev:/dev)"
    fail=1
fi

# The live /dev mount shows the host's GPU nodes with the host's permissions.
# The container user (this host user's uid) needs the ones the nvidia runtime
# would have created world-writable, or CUDA fails in the perception container.
gpu_missing=""
for node in /dev/nvhost-sched-gpu /dev/nvgpu/igpu0/sched /dev/nvhost-dbg-gpu; do
    [ -e "$node" ] || continue
    [ -r "$node" ] && [ -w "$node" ] || gpu_missing="$gpu_missing $node"
done
if [ -z "$gpu_missing" ]; then
    echo "ok   $(id -un) can open the GPU scheduler and debug nodes (configure_system.sh grants them)"
else
    echo "FAIL $(id -un) cannot open:$gpu_missing; CUDA in the container will fail with cudaErrorNotSupported (run ./configure_system.sh)"
    fail=1
fi

if [ $# -ge 1 ]; then
    container="$1"
    if ! cont_list=$(docker exec "$container" ls /dev 2>&1); then
        echo "FAIL cannot exec into container $container: $cont_list"
        exit 1
    fi
    host_nodes=$(ls /sys/class/video4linux | sort | tr '\n' ' ')
    cont_nodes=$(echo "$cont_list" | grep -E '^video[0-9]+$' | sort | tr '\n' ' ')
    if [ "$host_nodes" = "$cont_nodes" ]; then
        echo "ok   container $container sees the host's video nodes: $host_nodes"
    else
        echo "FAIL container $container video nodes differ from host"
        echo "     host:      $host_nodes"
        echo "     container: $cont_nodes"
        fail=1
    fi
fi

exit $fail
