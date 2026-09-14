"""A child process run in its own process group and stopped as a whole.

The perception bring-up is `ros2 launch`, which starts the perception container
and frame_rename.py as children of its own. Stopping it has to stop all of
them, including a container stuck in the camera driver that ignores SIGINT,
and the next start has to wait until none of them still holds the camera. So
the child gets a new session, making its process group everything it starts;
a stop sends SIGINT to the group and SIGKILL after stop_timeout_s; and the
child counts as exited only once no live member of the group is left.

If the supervisor dies without stopping the child (SIGKILL), the kernel sends
the child SIGINT, which ros2 launch treats as a clean shutdown of everything
it started. util-linux's setpriv sets that up and then execs the command; a
preexec_fn could deadlock in the forked child of a process that already runs
DDS threads. Linux only (/proc, setpriv).
"""
import os
import shutil
import signal
import subprocess
import time
from typing import Optional, Sequence

_SETPRIV = shutil.which('setpriv')


def live_group_members(pgid: int) -> int:
    """Processes in process group pgid that are not zombies."""
    count = 0
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            with open(f'/proc/{entry}/stat') as f:
                stat = f.read()
        except OSError:
            continue   # exited while we looked
        # The command name may contain spaces or parentheses; the fields that
        # follow its closing parenthesis are state, ppid, pgrp, ...
        state, _ppid, pgrp = stat[stat.rindex(')') + 2:].split()[:3]
        if int(pgrp) == pgid and state not in ('Z', 'X'):
            count += 1
    return count


class SupervisedProcess:
    def __init__(self, command: Sequence[str], stop_timeout_s: float):
        if not command:
            raise ValueError('a supervised process needs a command')
        self._command = list(command)
        self._stop_timeout_s = stop_timeout_s
        self._popen: Optional[subprocess.Popen] = None
        self._kill_at: Optional[float] = None   # set once a stop has been requested

    @property
    def command(self) -> Sequence[str]:
        return tuple(self._command)

    @property
    def pid(self) -> Optional[int]:
        """The child's pid, which is also its process group id; None when not running."""
        return self._popen.pid if self._popen is not None else None

    @property
    def running(self) -> bool:
        """True from start() until poll() has reported the exit."""
        return self._popen is not None

    def start(self) -> None:
        """Start the child. Raises OSError if the command cannot be run."""
        if self._popen is not None:
            raise RuntimeError('already running')
        # Checked here because behind setpriv a missing command is only an exit code.
        if shutil.which(self._command[0]) is None:
            raise FileNotFoundError(f'command not found: {self._command[0]}')
        wrapper = [_SETPRIV, '--pdeathsig', 'INT', '--'] if _SETPRIV else []
        self._popen = subprocess.Popen(wrapper + self._command, start_new_session=True)
        self._kill_at = None

    def request_stop(self, now: float) -> None:
        """SIGINT to the whole group now, SIGKILL from poll() after stop_timeout_s."""
        if self._popen is None or self._kill_at is not None:
            return
        self._signal(signal.SIGINT)
        self._kill_at = now + self._stop_timeout_s

    def poll(self, now: float) -> Optional[int]:
        """The child's return code, once, when it and everything it started are gone; else None."""
        if self._popen is None:
            return None
        leader_exited = self._popen.poll() is not None
        if not leader_exited or live_group_members(self._popen.pid):
            if leader_exited:
                self.request_stop(now)   # it left children behind; they go too
            if self._kill_at is not None and now >= self._kill_at:
                self._signal(signal.SIGKILL)
            return None
        returncode = self._popen.returncode
        self._popen = None
        self._kill_at = None
        return returncode

    def shutdown(self, timeout_s: float) -> Optional[int]:
        """Stop and wait, SIGKILL after at most timeout_s. For when the supervisor itself exits."""
        if self._popen is None:
            return None
        now = time.monotonic()
        self.request_stop(now)
        self._kill_at = min(self._kill_at, now + timeout_s)
        give_up = self._kill_at + 2.0
        while time.monotonic() < give_up:
            returncode = self.poll(time.monotonic())
            if returncode is not None:
                return returncode
            time.sleep(0.05)
        return None

    def _signal(self, sig: int) -> None:
        try:
            os.killpg(self._popen.pid, sig)
        except ProcessLookupError:
            pass
