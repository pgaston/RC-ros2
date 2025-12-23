#!/bin/bash
# Hotplug script for RealSense devices
# This script is triggered when a RealSense device is connected

if [ "$ACTION" == "add" ]; then
    echo "RealSense device connected: $DEVNAME"
elif [ "$ACTION" == "remove" ]; then
    echo "RealSense device disconnected: $DEVNAME"
fi