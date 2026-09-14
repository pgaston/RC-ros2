"""Goal relay through its ROS interface (testing seam one of the spec), against
a fake navigate_to_pose server that accepts or rejects, and succeeds, aborts
or reports Recoveries on command. Each test starts its own relay, so every
test begins with a relay that is not ready. Needs rclpy (the Isaac image).
"""
import math
import os
import pathlib
import signal
import subprocess
import sys
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster  # noqa: E402

from rc_hardware_control.goal_relay_policy import quaternion_of_yaw, yaw_of  # noqa: E402

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
RELAY = PACKAGE_ROOT / 'scripts' / 'goal_pose_relay.py'
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def wait_until(predicate, timeout_s, what='condition'):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError(f'timed out waiting for {what}')


class FakeNav2:
    """navigate_to_pose that does what the test tells it."""

    def __init__(self, node):
        self.accept = True
        self.goals = []        # NavigateToPose.Goal, in the order they arrived
        self.cancelled = []    # indexes of goals cancelled by the client
        self._commands = {}    # goal index -> (handle method, recoveries)
        self._executed = 0
        self._lock = threading.Lock()
        self._stopped = False
        self._server = ActionServer(
            node, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=lambda _handle: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def finish(self, index, how, recoveries=0):
        """End goal `index` with 'succeed' or 'abort', after reporting `recoveries`."""
        self._commands[index] = (how, recoveries)

    def stop(self):
        """End every goal still running, so the executor can shut down."""
        self._stopped = True

    def _on_goal(self, request):
        self.goals.append(request)
        return GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT

    def _execute(self, handle):
        with self._lock:
            index = self._executed
            self._executed += 1
        while True:
            if self._stopped:
                handle.abort()
                return NavigateToPose.Result()
            if handle.is_cancel_requested:
                self.cancelled.append(index)
                handle.canceled()
                return NavigateToPose.Result()
            command = self._commands.pop(index, None)
            if command is not None:
                how, recoveries = command
                if recoveries:
                    feedback = NavigateToPose.Feedback()
                    feedback.number_of_recoveries = recoveries
                    handle.publish_feedback(feedback)
                    time.sleep(0.3)
                getattr(handle, how)()
                return NavigateToPose.Result()
            time.sleep(0.02)


class Harness:
    """Everything around the relay: TF, the grid, perception status, the plan, Goal requests, Nav2."""

    def __init__(self):
        self.node = rclpy.create_node('goal_relay_test')
        self.nav = FakeNav2(self.node)
        self.statuses = []
        self.node.create_subscription(
            String, '/goal_relay/status', lambda m: self.statuses.append(m.data),
            QoSProfile(depth=20, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self._grid = self.node.create_publisher(OccupancyGrid, '/nvblox_node/static_occupancy_grid', 10)
        self._perception = self.node.create_publisher(String, '/perception/status', LATCHED)
        self._plan = self.node.create_publisher(Path, '/plan', 10)
        self.points = self.node.create_publisher(PointStamped, '/clicked_point', 10)
        self.poses = self.node.create_publisher(PoseStamped, '/goal_pose', 10)
        self._tf = StaticTransformBroadcaster(self.node)
        self._tf_dynamic = TransformBroadcaster(self.node)
        self._robot_on_tf = False
        self._transforms = []
        self._timers = []
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def robot_at(self, x, y, yaw):
        self.transform('odom', 'base_footprint', x, y, yaw)

    def transform(self, parent, child, x, y, yaw=0.0):
        self._transforms.append(self._transform_msg(parent, child, x, y, yaw))
        self._tf.sendTransform(self._transforms)

    def robot_on_tf(self, x, y, yaw, rate_hz=30.0):
        """odom to base_footprint on /tf at rate_hz, as visual SLAM publishes it, until stop_robot_on_tf()."""
        self._robot_on_tf = True

        def publish():
            if self._robot_on_tf:
                self._tf_dynamic.sendTransform(self._transform_msg('odom', 'base_footprint', x, y, yaw))
        self._timers.append(self.node.create_timer(1.0 / rate_hz, publish))

    def stop_robot_on_tf(self):
        # The timer keeps running: destroying it from the test thread while the
        # executor thread waits on it breaks the executor.
        self._robot_on_tf = False

    def _transform_msg(self, parent, child, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        r = t.transform.rotation
        r.x, r.y, r.z, r.w = quaternion_of_yaw(yaw)
        return t

    def grid(self):
        self._timers.append(self.node.create_timer(0.2, lambda: self._grid.publish(OccupancyGrid())))

    def perception(self, status):
        self._perception.publish(String(data=status))

    def plan(self):
        self._plan.publish(Path())

    def click(self, x, y, frame='odom'):
        msg = PointStamped()
        msg.header.frame_id = frame
        msg.point.x, msg.point.y = x, y
        self.points.publish(msg)

    def pose(self, x, y, yaw, frame='odom'):
        msg = PoseStamped()
        msg.header.frame_id = frame
        msg.pose.position.x, msg.pose.position.y = x, y
        q = msg.pose.orientation
        q.x, q.y, q.z, q.w = quaternion_of_yaw(yaw)
        self.poses.publish(msg)

    def ready(self, x=0.0, y=0.0, yaw=0.0):
        self.robot_at(x, y, yaw)
        self.grid()
        self.perception('healthy')
        time.sleep(1.0)

    def wait_for_statuses(self, count, timeout_s=5.0):
        wait_until(lambda: len(self.statuses) >= count, timeout_s, f'{count} statuses, have {self.statuses}')
        return list(self.statuses)

    def close(self):
        self.nav.stop()
        time.sleep(0.1)
        self._executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()


@pytest.fixture(scope='module')
def ros():
    os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def harness(ros):
    h = Harness()
    yield h
    h.close()


@pytest.fixture
def relay(harness):
    env = {**os.environ, 'PYTHONPATH': f"{PACKAGE_ROOT}{os.pathsep}{os.environ.get('PYTHONPATH', '')}"}
    process = subprocess.Popen([sys.executable, str(RELAY)], env=env)
    try:
        wait_until(lambda: harness.points.get_subscription_count() > 0
                   and harness.poses.get_subscription_count() > 0
                   and harness.node.count_subscribers('/navigate_to_pose/_action/feedback') > 0,
                   20.0, 'the relay to come up')
        time.sleep(0.5)
        yield process
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def assert_no_goal_sent(harness):
    time.sleep(0.5)
    assert harness.nav.goals == []


def only_goal(harness):
    wait_until(lambda: harness.nav.goals, 5.0, 'a goal at the fake Nav2')
    (goal,) = harness.nav.goals
    return goal.pose


def yaw_of_pose(pose):
    q = pose.pose.orientation
    return yaw_of(q.x, q.y, q.z, q.w)


def test_a_goal_before_the_transform_exists_is_rejected_and_not_sent(harness, relay):
    harness.grid()
    harness.perception('healthy')
    time.sleep(1.0)
    harness.click(3.0, 0.0)
    assert harness.wait_for_statuses(1) == ['rejected: no recent transform from odom to base_footprint']
    assert_no_goal_sent(harness)


def test_a_goal_before_any_occupancy_grid_is_rejected_and_not_sent(harness, relay):
    harness.robot_at(0.0, 0.0, 0.0)
    harness.perception('healthy')
    time.sleep(1.0)
    harness.click(3.0, 0.0)
    assert harness.wait_for_statuses(1) == ['rejected: no occupancy grid received yet']
    assert_no_goal_sent(harness)


def test_a_goal_while_perception_is_not_healthy_is_rejected_and_not_sent(harness, relay):
    harness.robot_at(0.0, 0.0, 0.0)
    harness.grid()
    harness.perception('stale: depth')
    time.sleep(1.0)
    harness.click(3.0, 0.0)
    assert harness.wait_for_statuses(1) == ['rejected: perception stale: depth']
    assert_no_goal_sent(harness)


def test_a_ready_goal_is_accepted_in_odom_facing_the_bearing_from_the_car(harness, relay):
    harness.ready(1.0, 1.0, 0.0)
    harness.click(4.0, 5.0)
    assert harness.wait_for_statuses(1)[0] == 'accepted: going to (4.00, 5.00) in odom, heading 53 deg'
    goal = only_goal(harness)
    assert goal.header.frame_id == 'odom'
    assert (goal.pose.position.x, goal.pose.position.y) == pytest.approx((4.0, 5.0))
    assert yaw_of_pose(goal) == pytest.approx(math.atan2(4.0, 3.0))


def test_a_car_pose_on_tf_is_used_like_a_static_one(harness, relay):
    harness.robot_on_tf(1.0, 1.0, 0.0)
    harness.grid()
    harness.perception('healthy')
    time.sleep(1.0)
    harness.click(4.0, 5.0)
    assert harness.wait_for_statuses(1)[0] == 'accepted: going to (4.00, 5.00) in odom, heading 53 deg'
    goal = only_goal(harness)
    assert yaw_of_pose(goal) == pytest.approx(math.atan2(4.0, 3.0))


def test_a_car_pose_that_stopped_arriving_is_not_ready(harness, relay):
    harness.robot_on_tf(1.0, 1.0, 0.0)
    harness.grid()
    harness.perception('healthy')
    time.sleep(1.0)
    harness.stop_robot_on_tf()
    time.sleep(2.5)   # longer than the relay keeps /tf
    harness.click(4.0, 5.0)
    assert harness.wait_for_statuses(1) == ['rejected: no recent transform from odom to base_footprint']
    assert_no_goal_sent(harness)


def test_a_pose_behaves_like_a_clicked_point(harness, relay):
    harness.ready(1.0, 1.0, 0.0)
    harness.pose(4.0, 5.0, yaw=2.5)
    assert harness.wait_for_statuses(1)[0] == 'accepted: going to (4.00, 5.00) in odom, heading 53 deg'
    goal = only_goal(harness)
    assert (goal.pose.position.x, goal.pose.position.y) == pytest.approx((4.0, 5.0))
    assert yaw_of_pose(goal) == pytest.approx(math.atan2(4.0, 3.0))


def test_a_goal_in_another_frame_is_transformed_into_odom(harness, relay):
    harness.transform('map', 'odom', 2.0, 0.0)
    harness.ready(0.0, 0.0, 0.0)
    harness.click(1.0, 0.0, frame='map')
    assert harness.wait_for_statuses(1)[0].startswith('accepted: going to (-1.00, 0.00) in odom')
    goal = only_goal(harness)
    assert (goal.pose.position.x, goal.pose.position.y) == pytest.approx((-1.0, 0.0))
    assert abs(yaw_of_pose(goal)) == pytest.approx(math.pi)


def test_a_goal_in_an_unknown_frame_is_rejected_and_not_sent(harness, relay):
    harness.ready()
    harness.click(1.0, 0.0, frame='nowhere')
    assert harness.wait_for_statuses(1)[0].startswith('rejected: cannot transform the Goal from nowhere to odom')
    assert_no_goal_sent(harness)


def test_a_second_goal_cancels_the_first_and_sends_the_second(harness, relay):
    harness.ready()
    harness.click(3.0, 0.0)
    harness.wait_for_statuses(1)
    harness.click(0.0, 3.0)
    statuses = harness.wait_for_statuses(3)
    assert statuses[1] == 'aborted: preempted by a new Goal'
    assert statuses[2].startswith('accepted: going to (0.00, 3.00)')
    wait_until(lambda: harness.nav.cancelled == [0], 5.0, 'the first goal to be cancelled')
    assert len(harness.nav.goals) == 2
    time.sleep(0.5)
    assert len(harness.statuses) == 3   # the first goal's cancellation is not reported twice


@pytest.mark.parametrize('how, recoveries, plan, expected', [
    ('succeed', 0, True, 'arrived: within the Arrival tolerance'),
    ('abort', 0, True, 'aborted: Nav2 aborted the Goal'),
    ('abort', 3, True, 'stuck: Recoveries did not free the car'),
    ('abort', 0, False, 'rejected: no path to the Goal'),
])
def test_how_nav2_ends_the_goal_is_the_status(harness, relay, how, recoveries, plan, expected):
    harness.ready()
    harness.click(3.0, 0.0)
    harness.wait_for_statuses(1)
    if plan:
        harness.plan()
        time.sleep(0.3)
    harness.nav.finish(0, how, recoveries)
    assert harness.wait_for_statuses(2)[1] == expected


def test_a_goal_that_ends_while_perception_is_lost_says_so(harness, relay):
    harness.ready()
    harness.click(3.0, 0.0)
    harness.wait_for_statuses(1)
    harness.plan()
    harness.perception('restarting: depth silent for 5 s')
    time.sleep(0.5)
    harness.nav.finish(0, 'abort')
    assert harness.wait_for_statuses(2)[1] == 'aborted: perception lost (restarting: depth silent for 5 s)'


def test_nav2_refusing_the_goal_is_a_rejection(harness, relay):
    harness.ready()
    harness.nav.accept = False
    harness.click(3.0, 0.0)
    assert harness.wait_for_statuses(1) == ['rejected: navigate_to_pose refused the Goal']
