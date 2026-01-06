# Library path for nvBlox
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ros/humble/share/isaac_ros_gxf/gxf/lib/serialization
# USB
sudo chmod 666 /dev/bus/usb/002/003
sudo chgrp plugdev /dev/bus/usb/002/003
sudo cp 99-realsense-libusb.rules /etc/udev/rules.d/
# Clean up
sudo rm /etc/udev/rules.d/99-realsense-libusb-custom.rules