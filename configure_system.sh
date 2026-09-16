#!/bin/bash
# Host setup that a reboot clears. Runs as root: at boot from
# rccar-host-setup.service (installed by scripts/install_host_setup.sh), or by
# hand, when it re-runs itself under sudo.
if [ "$(id -u)" -ne 0 ]; then
    exec sudo "$0" "$@"
fi
cd "$(dirname "$(readlink -f "$0")")"

echo "Configuring Jetson Performance and USB Settings..."

# 1. Disable USB Autosuspend (Critical for RealSense stability)
if [ -w /sys/module/usbcore/parameters/autosuspend ]; then
    echo "Disabling USB Autosuspend..."
    echo -1 > /sys/module/usbcore/parameters/autosuspend
else
    echo "Warning: Could not write to /sys/module/usbcore/parameters/autosuspend"
fi

# 2. Force Max Performance Mode (15W / Max Clocks)
# Note: nvpmodel -m 2 is usually persistent, but verifying it is good practice.
# The current model can be checked with 'nvpmodel -q'
echo "Setting NVP Model to 2 (15W Multi-Core)..."
nvpmodel -m 2

# 3. Maximize Clocks (Reset on reboot, so must run every time)
echo "Maximizing Jetson Clocks..."
jetson_clocks

# 4. Permissions for I2C and GPIO (Often needed after reboot)
echo "Setting I2C/GPIO Permissions..."
chmod 666 /dev/gpiochip* /dev/i2c* 2>/dev/null || true

# 5. Install RealSense UDEV Rules (One-time host setup)
if [ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then
    echo "Installing RealSense UDEV rules..."
    if [ -f ./99-realsense-libusb.rules ]; then
        cp ./99-realsense-libusb.rules /etc/udev/rules.d/
        udevadm control --reload-rules && udevadm trigger
        echo "Exectued udevadm trigger"
    else
        echo "Warning: 99-realsense-libusb.rules not found in current directory. Skipping."
    fi
fi

# 6. GPU device access for the container user (reset on reboot, so every time).
# The container bind-mounts the host's /dev live so the camera survives a
# re-enumeration (issue #9). That means the container sees the host's GPU
# device nodes with the host's permissions, not the world-writable copies the
# nvidia runtime would otherwise create, and the admin user in the container
# is the host user's uid. Without read/write on the scheduler and profiler
# nodes, CUDA reports "operation not supported" when NITROS sets up its memory
# pool and the perception container segfaults. Grant that user the nodes the
# runtime would have made 0666. The boot service names the user in RCCAR_USER.
CONTAINER_USER="${RCCAR_USER:-${SUDO_USER:-$(id -un)}}"
echo "Granting $CONTAINER_USER access to the GPU device nodes..."
for node in /dev/nvhost-*gpu* /dev/nvgpu/igpu0/* /dev/tegra-soc-hwpm /dev/dri/renderD128; do
    [ -e "$node" ] && setfacl -m "u:${CONTAINER_USER}:rw" "$node"
done

echo "System configuration complete. Ready for Docker."
