#!/bin/bash

echo "Configuring Jetson Performance and USB Settings..."

# 1. Disable USB Autosuspend (Critical for RealSense stability)
if [ -w /sys/module/usbcore/parameters/autosuspend ]; then
    echo "Disabling USB Autosuspend..."
    sudo sh -c 'echo -1 > /sys/module/usbcore/parameters/autosuspend'
else
    echo "Warning: Could not write to /sys/module/usbcore/parameters/autosuspend"
fi

# 2. Force Max Performance Mode (15W / Max Clocks)
# Note: nvpmodel -m 2 is usually persistent, but verifying it is good practice.
# The current model can be checked with 'nvpmodel -q'
echo "Setting NVP Model to 2 (15W Multi-Core)..."
sudo nvpmodel -m 2

# 3. Maximize Clocks (Reset on reboot, so must run every time)
echo "Maximizing Jetson Clocks..."
sudo jetson_clocks

# 4. Permissions for I2C and GPIO (Often needed after reboot)
echo "Setting I2C/GPIO Permissions..."
sudo chmod 666 /dev/gpiochip* /dev/i2c* 2>/dev/null || true

# 5. Install RealSense UDEV Rules (One-time host setup)
if [ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then
    echo "Installing RealSense UDEV rules..."
    if [ -f ./99-realsense-libusb.rules ]; then
        sudo cp ./99-realsense-libusb.rules /etc/udev/rules.d/
        sudo udevadm control --reload-rules && sudo udevadm trigger
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
# runtime would have made 0666.
CONTAINER_USER="${SUDO_USER:-$(id -un)}"
echo "Granting $CONTAINER_USER access to the GPU device nodes..."
for node in /dev/nvhost-*gpu* /dev/nvgpu/igpu0/* /dev/tegra-soc-hwpm /dev/dri/renderD128; do
    [ -e "$node" ] && sudo setfacl -m "u:${CONTAINER_USER}:rw" "$node"
done

echo "System configuration complete. Ready for Docker."
