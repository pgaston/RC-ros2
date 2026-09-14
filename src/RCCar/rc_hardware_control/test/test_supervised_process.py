"""Supervised process: a child in its own process group, stopped as a whole
with SIGINT and then SIGKILL, and reported exited only once nothing it started
is left. Real processes; Linux; runs on the host.
"""
import os
import pathlib
import subprocess
import sys
import time

import pytest

from rc_hardware_control.supervised_process import SupervisedProcess, live_group_members

PY = sys.executable
IGNORES_SIGINT = 'import signal, time\nsignal.signal(signal.SIGINT, signal.SIG_IGN)\ntime.sleep(30)'


def wait_for_exit(process, limit_s):
    end = time.monotonic() + limit_s
    while time.monotonic() < end:
        returncode = process.poll(time.monotonic())
        if returncode is not None:
            return returncode
        time.sleep(0.02)
    raise AssertionError(f'still running after {limit_s} s')


def test_a_child_that_exits_reports_its_return_code():
    p = SupervisedProcess([PY, '-c', 'raise SystemExit(3)'], stop_timeout_s=1.0)
    p.start()
    assert p.running
    assert wait_for_exit(p, 5.0) == 3
    assert not p.running


def test_stop_interrupts_a_child_that_handles_sigint():
    code = 'import time\ntry:\n    time.sleep(30)\nexcept KeyboardInterrupt:\n    raise SystemExit(0)'
    p = SupervisedProcess([PY, '-c', code], stop_timeout_s=5.0)
    p.start()
    time.sleep(0.5)   # let the interpreter come up
    p.request_stop(time.monotonic())
    assert wait_for_exit(p, 3.0) == 0


def test_a_child_that_ignores_sigint_is_killed_after_the_stop_timeout():
    p = SupervisedProcess([PY, '-c', IGNORES_SIGINT], stop_timeout_s=0.5)
    p.start()
    time.sleep(0.5)
    stopped_at = time.monotonic()
    p.request_stop(stopped_at)
    assert wait_for_exit(p, 3.0) == -9
    assert time.monotonic() - stopped_at >= 0.5


def test_exit_waits_for_everything_the_child_started():
    code = f'import subprocess, sys\nsubprocess.Popen([sys.executable, "-c", {IGNORES_SIGINT!r}])'
    p = SupervisedProcess([PY, '-c', code], stop_timeout_s=0.5)
    p.start()
    group = p.pid
    time.sleep(0.5)   # the child has exited; its own child is still running
    assert p.poll(time.monotonic()) is None
    assert live_group_members(group) == 1
    assert wait_for_exit(p, 3.0) == 0
    assert live_group_members(group) == 0


def test_shutdown_kills_within_its_own_timeout():
    p = SupervisedProcess([PY, '-c', IGNORES_SIGINT], stop_timeout_s=10.0)
    p.start()
    time.sleep(0.5)
    began = time.monotonic()
    p.shutdown(timeout_s=0.5)
    assert not p.running
    assert time.monotonic() - began < 2.0


def test_the_child_is_stopped_when_its_supervisor_is_killed(tmp_path):
    pid_file = tmp_path / 'child_pid'
    child = 'import time\ntime.sleep(30)'
    supervisor = (
        'import time\n'
        'from rc_hardware_control.supervised_process import SupervisedProcess\n'
        f'p = SupervisedProcess([{PY!r}, "-c", {child!r}], stop_timeout_s=1.0)\n'
        'p.start()\n'
        f'open({str(pid_file)!r}, "w").write(str(p.pid))\n'
        'time.sleep(30)\n'
    )
    env = {**os.environ, 'PYTHONPATH': str(pathlib.Path(__file__).resolve().parents[1])}
    outer = subprocess.Popen([PY, '-c', supervisor], env=env)
    try:
        end = time.monotonic() + 5.0
        while not (pid_file.exists() and pid_file.read_text()) and time.monotonic() < end:
            time.sleep(0.05)
        group = int(pid_file.read_text())
        time.sleep(0.5)
        assert live_group_members(group) == 1
        outer.kill()
        outer.wait()
        end = time.monotonic() + 3.0
        while live_group_members(group) and time.monotonic() < end:
            time.sleep(0.05)
        assert live_group_members(group) == 0
    finally:
        if outer.poll() is None:
            outer.kill()


def test_starting_twice_is_refused():
    p = SupervisedProcess([PY, '-c', 'import time; time.sleep(5)'], stop_timeout_s=0.5)
    p.start()
    try:
        with pytest.raises(RuntimeError):
            p.start()
    finally:
        p.shutdown(timeout_s=0.5)


def test_a_command_that_does_not_exist_raises():
    p = SupervisedProcess(['/nonexistent/perception'], stop_timeout_s=0.5)
    with pytest.raises(OSError):
        p.start()
    assert not p.running
