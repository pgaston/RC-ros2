#!/usr/bin/env python3
"""Soak recorder: the full launch over a long run, sampled every period_s (issue #14's one-hour test).

Run it next to the full launch, with the car still and no Goals:
  ros2 run rc_hardware_control soak_recorder.py
Every period_s it appends one row to soak_<start>.csv in output_dir:
  - message rates of depth camera_info, visual SLAM odometry and the nvblox
    occupancy grid (counted without decoding)
  - the perception watchdog's status
  - Jetson VDD_IN voltage, current and power, over-current throttle events,
    temperatures, load
  - the camera's USB address, which changes when the camera re-enumerates.
Every perception status change also goes to events_<start>.csv with its time.
Ctrl-C prints a summary; the first sample, taken while subscriptions are still
being discovered, is left out of its lowest rates.

The Jetson cannot see the battery voltage (see jetson_health.py). Read the
battery at the start and at the end.

Parameters: period_s, output_dir, and per stream <name>_topic for depth,
odometry and occupancy_grid.
"""
import csv
import datetime
import os
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from rc_hardware_control.jetson_health import TEMPERATURE_ZONES, read_health

STREAMS = {
    'depth': ('/depth/camera_info', CameraInfo),
    'odometry': ('/visual_slam/tracking/odometry', Odometry),
    'occupancy_grid': ('/nvblox_node/static_occupancy_grid', OccupancyGrid),
}
COLUMNS = (['time', 'elapsed_s'] + [f'{name}_hz' for name in STREAMS] + ['perception']
           + ['vdd_in_v', 'vdd_in_a', 'vdd_in_w', 'oc_events', 'current_alarm']
           + [f"{zone.split('-')[0]}_c" for zone in TEMPERATURE_ZONES] + ['load_1min', 'camera'])


def fmt(value, digits=2):
    if value is None:
        return ''
    return f'{value:.{digits}f}' if isinstance(value, float) else str(value)


class SoakRecorder(Node):
    def __init__(self):
        super().__init__('soak_recorder')
        self.declare_parameter('period_s', 10.0)
        self.declare_parameter('output_dir', '~/.ros/soak')
        self._period_s = self.get_parameter('period_s').value
        output_dir = os.path.expanduser(self.get_parameter('output_dir').value)
        os.makedirs(output_dir, exist_ok=True)
        started = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        self._rows = open(os.path.join(output_dir, f'soak_{started}.csv'), 'w', newline='', buffering=1)
        self._events = open(os.path.join(output_dir, f'events_{started}.csv'), 'w', newline='', buffering=1)
        self._row_writer = csv.writer(self._rows)
        self._event_writer = csv.writer(self._events)
        self._row_writer.writerow(COLUMNS)
        self._event_writer.writerow(['time', 'elapsed_s', 'perception'])

        self._counts = {name: 0 for name in STREAMS}
        for name, (topic, message_type) in STREAMS.items():
            self.declare_parameter(f'{name}_topic', topic)
            self.create_subscription(
                message_type, self.get_parameter(f'{name}_topic').value, self._counter(name),
                qos_profile_sensor_data, raw=True)
        self._perception = None
        self.create_subscription(
            String, '/perception/status', self._on_perception,
            QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

        self._began = time.monotonic()
        self._window_began = self._began
        first = read_health()
        self._summary = {
            'windows': 0, 'silent_windows': 0, 'status_changes': 0, 'restarts': 0,
            'camera_changes': 0, 'min_rates': {}, 'min_vdd_in_v': None, 'max_tj_c': None,
            'first_oc_events': first.oc_events, 'last_oc_events': first.oc_events,
            'last_camera': first.camera,
        }
        self.create_timer(self._period_s, self._on_period)
        self.get_logger().info(f'recording every {self._period_s:g} s to {output_dir} (soak_{started}.csv)')

    def _counter(self, name):
        def callback(_raw):
            self._counts[name] += 1
        return callback

    def _on_perception(self, msg: String):
        if msg.data == self._perception:
            return
        if self._perception is not None:
            self._summary['status_changes'] += 1
        if msg.data.startswith('restarting'):
            self._summary['restarts'] += 1
        self._perception = msg.data
        self._event_writer.writerow([datetime.datetime.now().isoformat(timespec='seconds'),
                                     f'{time.monotonic() - self._began:.1f}', msg.data])

    def _on_period(self):
        now = time.monotonic()
        window = now - self._window_began
        self._window_began = now
        rates = {name: count / window for name, count in self._counts.items()}
        self._counts = {name: 0 for name in STREAMS}
        h = read_health()
        s = self._summary

        s['windows'] += 1
        # The first window includes discovering the publishers, so its rates
        # are low for no reason; it is written to the CSV but kept out of the
        # summary's lowest rates and silent-stream count.
        if s['windows'] > 1:
            if any(rate == 0.0 for rate in rates.values()):
                s['silent_windows'] += 1
            for name, rate in rates.items():
                s['min_rates'][name] = min(rate, s['min_rates'].get(name, rate))
        if h.vdd_in_v is not None:
            s['min_vdd_in_v'] = h.vdd_in_v if s['min_vdd_in_v'] is None else min(h.vdd_in_v, s['min_vdd_in_v'])
        tj = h.temps_c.get('tj-thermal')
        if tj is not None:
            s['max_tj_c'] = tj if s['max_tj_c'] is None else max(tj, s['max_tj_c'])
        if h.oc_events is not None:
            s['last_oc_events'] = h.oc_events
        if h.camera != s['last_camera']:
            s['camera_changes'] += 1
            s['last_camera'] = h.camera

        self._row_writer.writerow(
            [datetime.datetime.now().isoformat(timespec='seconds'), f'{now - self._began:.1f}']
            + [fmt(rates[name], 1) for name in STREAMS] + [self._perception or '']
            + [fmt(h.vdd_in_v, 3), fmt(h.vdd_in_a, 3), fmt(h.vdd_in_w), fmt(h.oc_events), fmt(h.current_alarm)]
            + [fmt(h.temps_c.get(zone), 1) for zone in TEMPERATURE_ZONES]
            + [fmt(h.load_1min), h.camera or 'absent'])

        if s['windows'] % max(1, round(60.0 / self._period_s)) == 0:
            self.get_logger().info(
                f"{(now - self._began) / 60:.0f} min: depth {rates['depth']:.0f} Hz, "
                f"odometry {rates['odometry']:.0f} Hz, grid {rates['occupancy_grid']:.1f} Hz, "
                f"perception {self._perception}, VDD_IN {fmt(h.vdd_in_v, 2)} V {fmt(h.vdd_in_w, 1)} W, "
                f"tj {fmt(tj, 0)} C, camera {h.camera or 'absent'}")

    def summary(self) -> str:
        s = self._summary
        minutes = (time.monotonic() - self._began) / 60
        rates = ', '.join(f'{name} {rate:.1f} Hz' for name, rate in s['min_rates'].items())
        oc = (None if s['first_oc_events'] is None or s['last_oc_events'] is None
              else s['last_oc_events'] - s['first_oc_events'])
        return (f'soak: {minutes:.1f} min, {s["windows"]} samples, {s["silent_windows"]} with a silent stream; '
                f'lowest rates: {rates or "none"}; perception changes {s["status_changes"]}, '
                f'restarts {s["restarts"]}; camera USB changes {s["camera_changes"]}; '
                f'lowest VDD_IN {fmt(s["min_vdd_in_v"], 3)} V; over-current events {fmt(oc)}; '
                f'hottest tj {fmt(s["max_tj_c"], 1)} C')

    def close(self):
        self._rows.close()
        self._events.close()


def main(args=None):
    rclpy.init(args=args)
    node = SoakRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(node.summary(), flush=True)
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
