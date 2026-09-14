#!/usr/bin/env python3
"""Stand-in for the perception bring-up in test_perception_watchdog_node.py.

Publishes the three streams the watchdog watches at their real rates. On its
first run it goes silent after --stall-after seconds while staying alive and
ignoring SIGINT, like a camera container stuck in the driver; later runs keep
publishing. Each run appends a line to --runs-file.
"""
import argparse
import pathlib
import signal
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--runs-file', required=True)
    parser.add_argument('--stall-after', type=float, required=True)
    args, ros_args = parser.parse_known_args()

    runs = pathlib.Path(args.runs_file)
    first_run = not runs.exists()
    with runs.open('a') as f:
        f.write('run\n')

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    node = rclpy.create_node('fake_perception')
    depth = node.create_publisher(CameraInfo, '/depth/camera_info', 10)
    odometry = node.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
    grid = node.create_publisher(OccupancyGrid, '/nvblox_node/static_occupancy_grid', 10)

    began = time.monotonic()
    tick = 0
    while True:
        if not first_run or time.monotonic() - began < args.stall_after:
            depth.publish(CameraInfo())
            odometry.publish(Odometry())
            if tick % 6 == 0:
                grid.publish(OccupancyGrid())
        tick += 1
        time.sleep(1.0 / 30.0)


if __name__ == '__main__':
    main()
