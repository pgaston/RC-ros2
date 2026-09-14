"""Whether perception is healthy, and what the perception watchdog does when it is not.

Pure Python, no ROS: the node reports when the perception bring-up starts and
exits and when each watched stream publishes, and asks what to do now. A
stream silent for longer than its timeout is stale and the car is held. A
stream that published since the start and then stays silent for
restart_after_s, or one that has not published startup_grace_s after the
start, gets the bring-up stopped and started again. Losing perception, by that
stop or by an unexpected exit, cancels the Goal: visual SLAM restarts its odom
frame at the origin, where the Goal no longer means anything.

A start that follows a start which never became healthy resets the camera. On
the bench (2026-09-14) a camera left wedged by a USB reset failed every plain
restart with USB protocol errors on its depth interfaces; a restart alone does
not clear that, and a hardware reset is the next thing short of a replug. The
first start, and a restart after perception was healthy, do not reset: a reset
re-enumerates the camera and costs a few seconds.
"""
from dataclasses import dataclass
from typing import Dict, Iterable, NamedTuple, Optional, Tuple


@dataclass(frozen=True)
class Stream:
    name: str
    timeout_s: float     # silence longer than this is stale: hold the car


@dataclass(frozen=True)
class Timing:
    restart_after_s: float = 5.0    # silence after a stream came up that restarts perception
    startup_grace_s: float = 45.0   # time after a start for every stream to come up
    restart_delay_s: float = 3.0    # wait after an exit before the next start


class Decision(NamedTuple):
    hold: bool           # keep the car stopped
    start: bool          # start the perception bring-up now
    reset_camera: bool   # with start: the last start never became healthy, reset the camera
    stop: bool           # stop the running bring-up so it can start again
    cancel_goals: bool   # perception was lost; the Goal is in an odom frame about to reset
    status: str          # '<state>' or '<state>: <reason>'


class WatchdogPolicy:
    def __init__(self, streams: Iterable[Stream], timing: Timing = Timing()):
        streams = tuple(streams)
        if not streams:
            raise ValueError('a perception watchdog needs at least one stream')
        if len({s.name for s in streams}) != len(streams):
            raise ValueError('stream names must be distinct')
        for s in streams:
            if s.timeout_s <= 0.0:
                raise ValueError(f'stream {s.name!r} needs a positive timeout')
            if timing.restart_after_s < s.timeout_s:
                raise ValueError(f'restart_after_s is shorter than stream {s.name!r} timeout')
        if timing.startup_grace_s < timing.restart_after_s:
            raise ValueError('startup_grace_s is shorter than restart_after_s')
        if timing.restart_delay_s < 0.0:
            raise ValueError('restart_delay_s must not be negative')
        self._streams = streams
        self._timing = timing
        self._started_at: Optional[float] = None   # None while the bring-up is down
        self._exited_at: Optional[float] = None    # None until the first exit
        self._stopping = False
        self._down_reason = ''
        self._heard: Dict[str, float] = {}
        self._cancel_pending = False
        self._came_up = False        # this start has been healthy
        self._last_start_failed = False

    @property
    def streams(self) -> Tuple[Stream, ...]:
        return self._streams

    def started(self, now: float) -> None:
        """The bring-up was just started. Streams heard before this do not count."""
        self._started_at = now
        self._stopping = False
        self._came_up = False
        self._heard.clear()

    def exited(self, now: float, returncode: Optional[int]) -> None:
        """The bring-up and everything it started are gone. None: it could not be started."""
        if self._started_at is None:
            return
        if not self._stopping:
            self._cancel_pending = True
            self._down_reason = ('perception could not start' if returncode is None
                                 else f'perception exited with code {returncode}')
        self._last_start_failed = not self._came_up
        self._started_at = None
        self._exited_at = now
        self._stopping = False
        self._heard.clear()

    def heard(self, stream: str, now: float) -> None:
        """A watched stream published. Raises KeyError for an unknown stream."""
        if stream not in {s.name for s in self._streams}:
            raise KeyError(stream)
        if self._started_at is not None and not self._stopping:
            self._heard[stream] = now

    def decide(self, now: float) -> Decision:
        cancel, self._cancel_pending = self._cancel_pending, False

        if self._started_at is None:
            if self._exited_at is None or now - self._exited_at >= self._timing.restart_delay_s:
                if self._last_start_failed:
                    return Decision(True, True, True, False, cancel,
                                    'starting: resetting the camera, the last start never came up')
                return Decision(True, True, False, False, cancel, 'starting')
            return Decision(True, False, False, False, cancel, f'down: {self._down_reason}')

        if self._stopping:
            return Decision(True, False, False, False, cancel, f'restarting: {self._down_reason}')

        stale, waiting, restart_reason = [], [], None
        for s in self._streams:
            last = self._heard.get(s.name)
            if last is None:
                waiting.append(s.name)
                if restart_reason is None and now - self._started_at > self._timing.startup_grace_s:
                    restart_reason = f'{s.name} not up {self._timing.startup_grace_s:g} s after start'
            elif now - last > s.timeout_s:
                stale.append(s.name)
                if restart_reason is None and now - last > self._timing.restart_after_s:
                    restart_reason = f'{s.name} silent for {self._timing.restart_after_s:g} s'

        if restart_reason is not None:
            self._stopping = True
            self._down_reason = restart_reason
            return Decision(True, False, False, True, True, f'restarting: {restart_reason}')
        if stale:
            return Decision(True, False, False, False, cancel, 'stale: ' + ', '.join(stale))
        if waiting:
            return Decision(True, False, False, False, cancel, 'starting: waiting for ' + ', '.join(waiting))
        self._came_up = True
        return Decision(False, False, False, False, cancel, 'healthy')
