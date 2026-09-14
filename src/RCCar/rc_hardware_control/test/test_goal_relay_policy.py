"""Goal relay rules: admission reasons, the Goal heading, and the status value
for each way Nav2 can end a Goal. Pure Python; runs on the host.
"""
import math

import pytest

from rc_hardware_control.goal_relay_policy import (
    STATUS_ABORTED, STATUS_CANCELED, STATUS_SUCCEEDED, Readiness, goal_heading, outcome,
    quaternion_of_yaw, refusal, yaw_of)

READY = Readiness(server_available=True, robot_pose_known=True, grid_received=True,
                  perception_watched=True, perception_status='healthy')


def refuse(**changes):
    fields = {**READY.__dict__, **changes}
    return refusal(Readiness(**fields), 'navigate_to_pose', 'odom', 'base_footprint')


def test_a_ready_relay_admits():
    assert refuse() is None


@pytest.mark.parametrize('changes, reason', [
    (dict(server_available=False), 'navigate_to_pose is not available'),
    (dict(robot_pose_known=False), 'no transform from odom to base_footprint yet'),
    (dict(grid_received=False), 'no occupancy grid received yet'),
    (dict(perception_status=None), 'no perception status yet'),
    (dict(perception_status='stale: depth'), 'perception stale: depth'),
])
def test_each_missing_fact_is_the_reason(changes, reason):
    assert refuse(**changes) == reason


def test_the_first_missing_fact_is_reported():
    assert refuse(robot_pose_known=False, grid_received=False) == \
        'no transform from odom to base_footprint yet'


def test_perception_is_ignored_when_not_watched():
    assert refuse(perception_watched=False, perception_status=None) is None


@pytest.mark.parametrize('goal, heading', [
    ((4.0, 1.0), 0.0),
    ((1.0, 4.0), math.pi / 2),
    ((-2.0, 1.0), math.pi),
    ((1.0, -2.0), -math.pi / 2),
    ((4.0, 5.0), math.atan2(4.0, 3.0)),
])
def test_the_goal_faces_along_the_bearing_from_the_car(goal, heading):
    assert goal_heading(1.0, 1.0, 0.3, *goal) == pytest.approx(heading)


def test_a_goal_under_the_car_keeps_the_car_heading():
    assert goal_heading(1.0, 1.0, 0.3, 1.01, 1.02) == pytest.approx(0.3)


@pytest.mark.parametrize('yaw', [0.0, 1.0, -2.5, math.pi / 2])
def test_yaw_and_quaternion_round_trip(yaw):
    assert yaw_of(*quaternion_of_yaw(yaw)) == pytest.approx(yaw)


@pytest.mark.parametrize('status, recoveries, plan, perception, expected', [
    (STATUS_SUCCEEDED, 0, True, 'healthy', 'arrived'),
    (STATUS_SUCCEEDED, 3, True, 'stale: depth', 'arrived'),
    (STATUS_ABORTED, 3, True, 'healthy', 'stuck'),
    (STATUS_ABORTED, 0, False, 'healthy', 'rejected'),
    (STATUS_ABORTED, 0, True, 'healthy', 'aborted'),
    (STATUS_ABORTED, 3, True, 'restarting: depth silent for 5 s', 'aborted'),
    (STATUS_CANCELED, 0, True, 'restarting: depth silent for 5 s', 'aborted'),
    (STATUS_CANCELED, 0, True, 'healthy', 'aborted'),
    (STATUS_ABORTED, 0, False, None, 'rejected'),
    (2, 0, True, 'healthy', 'aborted'),
])
def test_how_nav2_ended_the_goal_names_the_status(status, recoveries, plan, perception, expected):
    value, reason = outcome(status, recoveries, plan, perception)
    assert value == expected
    assert reason


def test_perception_lost_is_the_reason_when_perception_is_not_healthy():
    assert outcome(STATUS_CANCELED, 0, True, 'restarting: depth silent for 5 s') == \
        ('aborted', 'perception lost (restarting: depth silent for 5 s)')


def test_no_path_is_the_reason_for_a_goal_nav2_never_planned():
    assert outcome(STATUS_ABORTED, 0, False, 'healthy') == ('rejected', 'no path to the Goal')
