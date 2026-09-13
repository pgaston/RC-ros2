"""Velocity mux selection: teleop beats Nav2, a silent source drops out after
its timeout, and no fresh source means stop. Pure Python; runs on the host.
"""
import pytest

from rc_hardware_control.velocity_mux_policy import MuxPolicy, Source

TELEOP = Source(name='teleop', priority=100, timeout_s=0.5)
NAV = Source(name='nav', priority=10, timeout_s=0.5)


@pytest.fixture
def mux():
    return MuxPolicy([NAV, TELEOP])


def test_nothing_offered_selects_nothing(mux):
    assert mux.select(now=0.0) is None


def test_fresh_nav_alone_drives(mux):
    mux.offer('nav', 'nav-cmd', now=1.0)
    assert mux.select(now=1.2) == ('nav', 'nav-cmd')


def test_teleop_takes_over_from_nav_immediately(mux):
    mux.offer('nav', 'nav-cmd', now=1.0)
    mux.offer('teleop', 'teleop-cmd', now=1.1)
    assert mux.select(now=1.1) == ('teleop', 'teleop-cmd')


def test_priority_wins_regardless_of_arrival_order(mux):
    mux.offer('teleop', 'teleop-cmd', now=1.0)
    mux.offer('nav', 'nav-cmd', now=1.1)
    assert mux.select(now=1.2) == ('teleop', 'teleop-cmd')


def test_released_teleop_hands_back_to_nav_after_its_timeout(mux):
    mux.offer('teleop', 'teleop-cmd', now=1.0)
    mux.offer('nav', 'nav-cmd', now=1.4)
    assert mux.select(now=1.49) == ('teleop', 'teleop-cmd')
    assert mux.select(now=1.51) == ('nav', 'nav-cmd')


def test_silent_nav_stops_the_car(mux):
    mux.offer('nav', 'nav-cmd', now=1.0)
    assert mux.select(now=1.5) == ('nav', 'nav-cmd')
    assert mux.select(now=1.51) is None


def test_timeout_is_measured_from_the_latest_message(mux):
    mux.offer('nav', 'first', now=1.0)
    mux.offer('nav', 'second', now=1.4)
    assert mux.select(now=1.8) == ('nav', 'second')


def test_unknown_source_is_rejected(mux):
    with pytest.raises(KeyError):
        mux.offer('joystick', 'cmd', now=0.0)


def test_source_timeout_must_be_positive():
    with pytest.raises(ValueError):
        MuxPolicy([Source(name='nav', priority=1, timeout_s=0.0)])


def test_priorities_must_be_distinct():
    with pytest.raises(ValueError):
        MuxPolicy([Source('a', 5, 1.0), Source('b', 5, 1.0)])
