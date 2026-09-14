"""Perception watchdog decisions: hold the car while perception is not healthy,
restart the bring-up when a stream stalls or never comes up, and cancel the
Goal when perception is lost. Pure Python; runs on the host.
"""
import pytest

from rc_hardware_control.perception_watchdog_policy import Stream, Timing, WatchdogPolicy

DEPTH = Stream('depth', timeout_s=0.5)
GRID = Stream('grid', timeout_s=2.0)
TIMING = Timing(restart_after_s=5.0, startup_grace_s=45.0, restart_delay_s=3.0)


@pytest.fixture
def dog():
    return WatchdogPolicy([DEPTH, GRID], TIMING)


def healthy_at(dog, now):
    dog.started(now=0.0)
    dog.heard('depth', now)
    dog.heard('grid', now)
    return dog


def test_first_decision_starts_perception_and_holds_the_car(dog):
    d = dog.decide(now=0.0)
    assert d.start and d.hold
    assert not d.stop and not d.cancel_goals


def test_the_car_is_held_until_every_stream_has_published(dog):
    dog.started(now=0.0)
    dog.heard('depth', now=1.0)
    d = dog.decide(now=1.1)
    assert d.hold and not d.start and not d.stop
    assert d.status == 'starting: waiting for grid'


def test_all_streams_fresh_releases_the_car(dog):
    healthy_at(dog, 10.0)
    assert dog.decide(now=10.1) == (False, False, False, False, False, 'healthy')


def test_a_stale_stream_holds_the_car_without_restarting(dog):
    healthy_at(dog, 10.0)
    dog.heard('grid', now=10.5)
    d = dog.decide(now=10.6)   # depth silent 0.6 s, over its 0.5 s
    assert d.hold and not d.stop and not d.cancel_goals
    assert d.status == 'stale: depth'


def test_a_stream_back_before_restart_after_releases_the_car(dog):
    healthy_at(dog, 10.0)
    assert dog.decide(now=12.0).hold
    dog.heard('depth', now=12.1)
    dog.heard('grid', now=12.1)
    assert not dog.decide(now=12.2).hold


def test_a_stalled_stream_stops_perception_once_and_cancels_goals(dog):
    healthy_at(dog, 10.0)
    dog.heard('grid', now=15.0)
    d = dog.decide(now=15.1)   # depth silent 5.1 s
    assert d.stop and d.hold and d.cancel_goals and not d.start
    assert d.status == 'restarting: depth silent for 5 s'
    again = dog.decide(now=15.2)
    assert again.hold and not again.stop and not again.cancel_goals
    assert again.status == d.status


def test_a_stream_that_never_comes_up_gets_the_startup_grace(dog):
    dog.started(now=0.0)
    dog.heard('depth', now=44.8)
    assert not dog.decide(now=44.9).stop
    d = dog.decide(now=45.1)
    assert d.stop and d.status == 'restarting: grid not up 45 s after start'


def test_the_next_start_waits_for_the_restart_delay(dog):
    healthy_at(dog, 10.0)
    assert dog.decide(now=16.0).stop
    dog.exited(now=20.0, returncode=0)
    d = dog.decide(now=22.9)
    assert d.hold and not d.start and not d.cancel_goals
    assert d.status == 'down: depth silent for 5 s'
    assert dog.decide(now=23.0).start


def test_an_unexpected_exit_cancels_goals_then_restarts(dog):
    healthy_at(dog, 10.0)
    dog.exited(now=11.0, returncode=-11)
    d = dog.decide(now=11.05)
    assert d.cancel_goals and d.hold and not d.start
    assert d.status == 'down: perception exited with code -11'
    later = dog.decide(now=14.0)
    assert later.start and not later.cancel_goals


def test_a_command_that_cannot_start_is_retried(dog):
    dog.decide(now=0.0)
    dog.started(now=0.0)
    dog.exited(now=0.0, returncode=None)
    assert dog.decide(now=1.0).status == 'down: perception could not start'
    assert dog.decide(now=3.0).start


def test_streams_heard_before_a_restart_do_not_count(dog):
    healthy_at(dog, 10.0)
    dog.decide(now=16.0)
    dog.exited(now=17.0, returncode=0)
    dog.started(now=20.0)
    d = dog.decide(now=20.1)
    assert d.hold and d.status == 'starting: waiting for depth, grid'


def test_streams_heard_while_stopping_do_not_release_the_car(dog):
    healthy_at(dog, 10.0)
    dog.decide(now=16.0)
    dog.heard('depth', now=16.5)
    dog.heard('grid', now=16.5)
    d = dog.decide(now=16.6)
    assert d.hold and d.status.startswith('restarting')


def never_came_up(dog, now):
    """Start, let the startup grace run out, and let the stop finish."""
    d = dog.decide(now)
    assert d.start
    dog.started(now)
    assert dog.decide(now + 45.1).stop
    dog.exited(now + 46.0, returncode=0)
    return now + 46.0


def test_the_first_start_does_not_reset_the_camera(dog):
    d = dog.decide(now=0.0)
    assert d.start and not d.reset_camera and d.status == 'starting'


def test_a_start_after_a_start_that_never_came_up_resets_the_camera(dog):
    exited = never_came_up(dog, 0.0)
    assert not dog.decide(now=exited + 2.9).start
    d = dog.decide(now=exited + 3.0)
    assert d.start and d.reset_camera
    assert d.status == 'starting: resetting the camera, the last start never came up'


def test_a_bring_up_that_exits_before_it_came_up_resets_the_camera_next(dog):
    dog.decide(now=0.0)
    dog.started(now=0.0)
    dog.heard('depth', now=5.0)
    dog.exited(now=6.0, returncode=-11)   # a missing camera ends in a segfault
    d = dog.decide(now=9.0)
    assert d.start and d.reset_camera


def test_a_restart_after_perception_was_healthy_does_not_reset_the_camera(dog):
    healthy_at(dog, 10.0)
    assert not dog.decide(now=10.1).hold
    assert dog.decide(now=16.0).stop
    dog.exited(now=17.0, returncode=0)
    d = dog.decide(now=20.0)
    assert d.start and not d.reset_camera


def test_the_camera_is_reset_on_every_start_until_one_comes_up(dog):
    exited = never_came_up(dog, 0.0)
    exited = never_came_up(dog, exited + 3.0)
    d = dog.decide(now=exited + 3.0)
    assert d.start and d.reset_camera
    dog.started(now=exited + 3.0)
    dog.heard('depth', now=exited + 20.0)
    dog.heard('grid', now=exited + 20.0)
    assert dog.decide(now=exited + 20.1).status == 'healthy'
    dog.exited(now=exited + 30.0, returncode=-11)
    d = dog.decide(now=exited + 33.0)
    assert d.start and not d.reset_camera


def test_only_the_start_decision_asks_for_a_reset(dog):
    exited = never_came_up(dog, 0.0)
    assert not dog.decide(now=exited + 1.0).reset_camera   # still down
    assert dog.decide(now=exited + 3.0).reset_camera
    dog.started(now=exited + 3.0)
    assert not dog.decide(now=exited + 3.1).reset_camera


def test_unknown_stream_is_rejected(dog):
    with pytest.raises(KeyError):
        dog.heard('lidar', now=0.0)


@pytest.mark.parametrize('streams, timing', [
    ([], TIMING),
    ([DEPTH, Stream('depth', 1.0)], TIMING),
    ([Stream('depth', 0.0)], TIMING),
    ([Stream('slow', 6.0)], TIMING),
    ([DEPTH], Timing(restart_after_s=5.0, startup_grace_s=4.0)),
    ([DEPTH], Timing(restart_delay_s=-1.0)),
])
def test_inconsistent_configuration_is_rejected(streams, timing):
    with pytest.raises(ValueError):
        WatchdogPolicy(streams, timing)
