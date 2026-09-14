"""Cross-track error and per-Goal summaries for Path follower tuning (#7).
Pure Python; runs on the host.
"""
import math

import pytest

from rc_hardware_control.tracking_error import GoalSummary, GoalTrack, distance_to_path, percentile

STRAIGHT = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]


@pytest.mark.parametrize('point, distance', [
    ((1.5, 0.2), 0.2),          # beside a segment
    ((1.5, -0.3), 0.3),         # either side
    ((4.0, 0.0), 1.0),          # past the end
    ((-0.5, 0.0), 0.5),         # before the start
    ((2.0, 0.0), 0.0),          # on a vertex
])
def test_distance_to_a_straight_path(point, distance):
    assert distance_to_path(*point, STRAIGHT) == pytest.approx(distance)


def test_distance_to_a_bent_path_is_to_the_nearest_segment():
    bent = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert distance_to_path(2.0, 0.5, bent) == pytest.approx(1.0)
    assert distance_to_path(0.5, 0.5, bent) == pytest.approx(0.5)


def test_distance_to_a_one_point_or_empty_path():
    assert distance_to_path(3.0, 4.0, [(0.0, 0.0)]) == pytest.approx(5.0)
    assert distance_to_path(3.0, 4.0, []) == math.inf


def test_repeated_points_in_a_path_do_not_break_the_distance():
    assert distance_to_path(0.5, 0.2, [(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)]) == pytest.approx(0.2)


def test_percentile_is_nearest_rank():
    assert percentile(list(range(1, 21)), 95) == 19
    assert percentile([0.3], 95) == 0.3
    assert math.isnan(percentile([], 95))


def test_a_goal_summary_measures_against_the_latest_and_the_first_plan():
    track = GoalTrack(goal=1, started_at=10.0, limit_m=0.15)
    track.plan(STRAIGHT)
    track.position(1.0, 0.1)
    track.plan([(2.0, 0.2), (3.0, 0.2)])    # replanned from where the car is
    track.position(2.0, 0.2)
    track.position(2.5, 0.06)
    summary = track.finish(ended_at=16.0, outcome='arrived: within the Arrival tolerance')
    assert summary.samples == 3
    assert summary.duration_s == pytest.approx(6.0)
    assert summary.max_error_m == pytest.approx(0.14)
    assert summary.mean_error_m == pytest.approx((0.1 + 0.0 + 0.14) / 3)
    assert summary.within_limit == pytest.approx(1.0)
    assert summary.max_error_first_plan_m == pytest.approx(0.2)
    assert summary.travelled_m == pytest.approx(math.hypot(1.0, 0.1) + math.hypot(0.5, 0.14))


def test_positions_before_any_plan_count_as_travel_but_not_as_samples():
    track = GoalTrack(goal=2, started_at=0.0, limit_m=0.15)
    assert track.position(0.0, 0.0) is None
    assert track.position(0.5, 0.0) is None
    track.plan(STRAIGHT)
    assert track.position(1.0, 0.3) == pytest.approx((0.3, 0.3))
    summary = track.finish(ended_at=2.0, outcome='stuck: Recoveries did not free the car')
    assert summary.samples == 1
    assert summary.within_limit == 0.0
    assert summary.travelled_m == pytest.approx(0.5 + math.hypot(0.5, 0.3))


def test_a_goal_with_no_samples_has_no_error_numbers():
    track = GoalTrack(goal=3, started_at=0.0, limit_m=0.15)
    track.plan([])
    summary = track.finish(ended_at=2.5, outcome='rejected: no path to the Goal')
    assert summary.samples == 0
    assert math.isnan(summary.max_error_m) and math.isnan(summary.within_limit)
    assert 'no position measured' in summary.describe()


def test_a_summary_row_matches_its_header():
    track = GoalTrack(goal=4, started_at=0.0, limit_m=0.15)
    track.plan(STRAIGHT)
    track.position(1.0, 0.1)
    summary = track.finish(ended_at=1.0, outcome='arrived: x')
    assert len(summary.row()) == len(GoalSummary.header())
    assert summary.row()[0] == '4' and summary.row()[5] == '0.1000'
