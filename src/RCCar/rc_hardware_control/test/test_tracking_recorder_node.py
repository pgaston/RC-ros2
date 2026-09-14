"""Tracking recorder through its ROS interface: a Goal accepted, a plan, three
car positions and a final status produce the published errors, the samples
file and one summary row. Needs rclpy (the Isaac image); about 10 s.
"""
import csv
import os
import pathlib
import signal
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PoseStamped  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from std_msgs.msg import Float32, String  # noqa: E402

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
RECORDER = PACKAGE_ROOT / 'scripts' / 'tracking_recorder.py'


def wait_until(predicate, timeout_s, what):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError(f'timed out waiting for {what}')


def test_a_goal_is_measured_and_summarised(tmp_path):
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    env = {**os.environ, 'PYTHONPATH': f"{PACKAGE_ROOT}{os.pathsep}{os.environ.get('PYTHONPATH', '')}"}
    recorder = subprocess.Popen([
        sys.executable, str(RECORDER), '--ros-args',
        '-p', f'output_dir:={tmp_path}', '-p', 'limit_m:=0.15',
    ], env=env)
    rclpy.init()
    node = rclpy.create_node('tracking_recorder_test')
    try:
        status = node.create_publisher(String, '/goal_relay/status', 10)
        plan = node.create_publisher(Path, '/plan', 10)
        odometry = node.create_publisher(Odometry, '/visual_slam/tracking/odometry', 10)
        errors = []
        node.create_subscription(Float32, '/tracking/cross_track_error', lambda m: errors.append(m.data), 10)

        def spin_for(seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)

        wait_until(lambda: all(p.get_subscription_count() for p in (status, plan, odometry)), 20.0,
                   'the recorder to subscribe')
        spin_for(0.5)

        status.publish(String(data='accepted: going to (3.00, 0.00) in odom, heading 0 deg'))
        spin_for(0.3)
        path = Path()
        path.header.frame_id = 'odom'
        for x in (0.0, 1.0, 2.0, 3.0):
            pose = PoseStamped()
            pose.pose.position.x = x
            path.poses.append(pose)
        plan.publish(path)
        spin_for(0.3)
        for x, y in ((1.0, 0.1), (2.0, 0.2), (2.5, 0.05)):
            msg = Odometry()
            msg.header.frame_id = 'odom'
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            odometry.publish(msg)
            spin_for(0.15)
        status.publish(String(data='arrived: within the Arrival tolerance'))
        spin_for(0.5)

        assert errors == pytest.approx([0.1, 0.2, 0.05])
        wait_until(lambda: list(tmp_path.glob('goals_*.csv'))
                   and len(list(tmp_path.glob('goals_*.csv'))[0].read_text().splitlines()) == 2,
                   5.0, 'the summary row')
        (goals,) = tmp_path.glob('goals_*.csv')
        row = list(csv.DictReader(goals.open()))[0]
        assert row['goal'] == '1'
        assert row['outcome'] == 'arrived: within the Arrival tolerance'
        assert row['samples'] == '3'
        assert float(row['max_error_m']) == pytest.approx(0.2)
        assert float(row['within_limit']) == pytest.approx(2 / 3, abs=1e-4)
        (samples,) = tmp_path.glob('samples_*.csv')
        assert len(samples.read_text().splitlines()) == 4
    finally:
        recorder.send_signal(signal.SIGINT)
        try:
            recorder.wait(timeout=5)
        except subprocess.TimeoutExpired:
            recorder.kill()
        node.destroy_node()
        rclpy.try_shutdown()
