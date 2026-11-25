#!/bin/bash
# kill_after.sh GRACE_S COMMAND [ARGS...]
#
# Runs COMMAND. On SIGINT or SIGTERM, passes the signal on, then SIGKILLs
# COMMAND if it is still running GRACE_S seconds later.
#
# For foxglove_bridge 3.2.4, which can hang on shutdown with a client
# connected: Ctrl-C left `ros2 launch` waiting on it for minutes, and launch
# never escalated to SIGTERM/SIGKILL (2026-11-25). Used as a launch prefix.

grace="$1"
shift

# Job control: the child gets its own process group and a normal SIGINT.
# Without it bash starts a background child with SIGINT ignored. The
# terminal's Ctrl-C then reaches only this script, which passes it on once.
set -m
"$@" &
child=$!

stop() {
  kill "-$1" "$child" 2>/dev/null
  ( sleep "$grace"; kill -KILL "$child" 2>/dev/null ) &
  trap '' INT TERM   # one timer; further Ctrl-Cs are ignored, not fatal
}
trap 'stop INT' INT
trap 'stop TERM' TERM

# A trapped signal interrupts wait, so loop until the child has really gone.
while kill -0 "$child" 2>/dev/null; do
  wait "$child"
  status=$?
done
exit "${status:-0}"
