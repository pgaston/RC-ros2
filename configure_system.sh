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

echo "System configuration complete. Ready for Docker."
