"""The car-like navigate-to-pose tree: a straight reverse and a pause are the
only behaviours, three Recoveries then abort, planning failure is not
recovered. Shape only; the numbers are judged on the patio. Pure Python.
"""
import pathlib
import xml.etree.ElementTree as ET

import pytest

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
TREE = PACKAGE_ROOT / 'behavior_trees' / 'navigate_to_pose_car_like.xml'

BEHAVIOUR_NODES = {'Spin', 'BackUp', 'DriveOnHeading', 'AssistedTeleop', 'Wait'}


@pytest.fixture(scope='module')
def tree():
    return ET.parse(TREE).getroot()


@pytest.fixture(scope='module')
def nodes(tree):
    return list(tree.iter())


def test_reverse_and_pause_are_the_only_behaviours(nodes):
    used = {n.tag for n in nodes if n.tag in BEHAVIOUR_NODES}
    assert used == {'BackUp', 'Wait'}


def test_three_recoveries_then_abort(nodes):
    recovery = [n for n in nodes if n.tag == 'RecoveryNode']
    assert len(recovery) == 1
    assert recovery[0].get('number_of_retries') == '3'
    assert recovery[0][0].tag == 'FollowPath', 'Recovery wraps path following only'


def test_planner_sits_outside_the_recovery(tree):
    # A planning failure fails the pipeline at once instead of reversing the car.
    recovery = tree.find('.//RecoveryNode')
    assert recovery.find('.//ComputePathToPose') is None
    pipeline = tree.find('.//PipelineSequence')
    assert pipeline[0].find('.//ComputePathToPose') is not None
    assert pipeline[1] is recovery


def test_recovery_is_settle_reverse_then_pause(nodes):
    sequence = next(n for n in nodes if n.tag == 'Sequence')
    assert [child.tag for child in sequence] == ['Wait', 'ForceSuccess', 'Wait']
    assert [child.tag for child in sequence[1]] == ['BackUp']


def test_a_cut_short_reverse_still_counts_as_a_recovery(nodes):
    # A failed recovery child makes Nav2's RecoveryNode abort the Goal at once;
    # the reverse is best effort, so its failure is swallowed.
    backup = next(n for n in nodes if n.tag == 'BackUp')
    parent = next(n for n in nodes if backup in list(n))
    assert parent.tag == 'ForceSuccess'


def test_reverse_is_straight(nodes):
    backup = next(n for n in nodes if n.tag == 'BackUp')
    assert float(backup.get('backup_dist')) > 0
    assert float(backup.get('backup_speed')) > 0


def test_a_new_goal_interrupts_recovery(tree):
    fallback = tree.find('.//ReactiveFallback')
    assert fallback[0].tag == 'GoalUpdated'
