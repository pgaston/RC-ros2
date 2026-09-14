"""Perception watchdog through its ROS interface, against fake_perception.py:
a bring-up that publishes the watched streams, stalls while ignoring SIGINT,
and publishes again once restarted. Asserts the status sequence, that the car
is held exactly while perception is not healthy, that Goals are cancelled once
on the restart, and that nothing is left running after the watchdog exits.
Needs rclpy (the Isaac image); about 25 s.
"""
import os
import pathlib
import signal
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from action_msgs.srv import CancelGoal  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402

TEST_DIR = pathlib.Path(__file__).resolve().parent
PACKAGE_ROOT = TEST_DIR.parent
WATCHDOG = PACKAGE_ROOT / 'scripts' / 'perception_watchdog.py'
FAKE = TEST_DIR / 'fake_perception.py'


def ros_list(items):
    return '[' + ', '.join(f"'{i}'" for i in items) + ']'


@pytest.fixture
def ros_env(tmp_path):
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    env['PYTHONPATH'] = f"{PACKAGE_ROOT}{os.pathsep}{env.get('PYTHONPATH', '')}"
    os.environ['ROS_DOMAIN_ID'] = env['ROS_DOMAIN_ID']
    return env


def test_a_stalled_bring_up_is_held_restarted_and_cleaned_up(tmp_path, ros_env):
    runs_file = tmp_path / 'runs'
    command = [sys.executable, str(FAKE), '--runs-file', str(runs_file), '--stall-after', '4']
    watchdog = subprocess.Popen([
        sys.executable, str(WATCHDOG), '--ros-args',
        '-p', f'command:={ros_list(command)}',
        '-p', 'restart_after_s:=2.0',
        '-p', 'startup_grace_s:=10.0',
        '-p', 'restart_delay_s:=1.0',
        '-p', 'stop_timeout_s:=1.0',
    ], env=ros_env)

    rclpy.init()
    node = rclpy.create_node('watchdog_test')
    statuses, holds, cancels = [], [], []
    node.create_subscription(
        String, '/perception/status', lambda m: statuses.append((time.monotonic(), m.data)),
        QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Twist, '/cmd_vel_hold', lambda m: holds.append(time.monotonic()), 50)

    def on_cancel(request, response):
        cancels.append(time.monotonic())
        return response
    node.create_service(CancelGoal, '/navigate_to_pose/_action/cancel_goal', on_cancel)

    try:
        deadline = time.monotonic() + 40.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            healthy_count = sum(1 for _, s in statuses if s == 'healthy')
            if healthy_count >= 2 and time.monotonic() - statuses[-1][0] > 1.0:
                break
        sequence = [s.split(':')[0] for _, s in statuses]
        assert healthy_count >= 2, statuses

        first_healthy = sequence.index('healthy')
        after = sequence[first_healthy:]
        assert after[:2] == ['healthy', 'stale'], statuses
        # Every stream stops at once; whichever published last longest ago restarts it.
        assert any(s.startswith('restarting: ') and s.endswith(' silent for 2 s')
                   for _, s in statuses), statuses
        restart = after.index('restarting')
        assert after[restart + 1] == 'down', statuses
        assert 'healthy' in after[restart:], statuses

        assert runs_file.read_text().count('run') == 2
        assert len(cancels) == 1

        # Held exactly while not healthy: no hold inside a healthy window, some outside.
        changes = statuses + [(time.monotonic(), 'end')]
        for (t, status), (t_next, _) in zip(changes, changes[1:]):
            inside = [h for h in holds if t + 0.2 < h < t_next - 0.2]
            if status == 'healthy':
                assert not inside, (status, t, inside[:3])
            elif t_next - t > 0.5:
                assert inside, (status, t)
    finally:
        watchdog.send_signal(signal.SIGINT)
        try:
            watchdog.wait(timeout=15)
        finally:
            if watchdog.poll() is None:
                watchdog.kill()
        node.destroy_node()
        rclpy.try_shutdown()

    leftovers = subprocess.run(['pgrep', '-f', str(runs_file)], capture_output=True, text=True)
    assert leftovers.stdout == '', leftovers.stdout
