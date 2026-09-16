"""DDS inside the container: loopback only, no shared memory (issue #17).

With ROS_LOCALHOST_ONLY=1, Humble's rmw_fastrtps adds a shared-memory transport
whatever the XML profile says, and the bridge's shared-memory listener spun at
80% of a core. The container therefore runs with ROS_LOCALHOST_ONLY=0 and this
profile, which keeps UDP on 127.0.0.1. Needs rclpy (the Isaac image); about 5 s.
"""
import os
import pathlib
import socket
import struct
import subprocess
import sys
import time

import pytest

pytest.importorskip('rclpy')

PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE_DIR.parents[2]
PROFILE = PACKAGE_DIR / 'config' / 'disable_shm.xml'
CONTAINER_PROFILE = '/workspaces/isaac_ros-dev/src/RCCar/rc_hardware_control/config/disable_shm.xml'
PARTICIPANT = '''
import os, socket, time, rclpy
from std_msgs.msg import String
rclpy.init()
node = rclpy.create_node('dds_profile_probe')
node.create_publisher(String, '/dds_profile_probe', 10)
# The receive buffer of each UDP socket this process listens on: the ones in
# the domain's RTPS port range, not the ephemeral ports it only sends from.
base = 7400 + 250 * int(os.environ['ROS_DOMAIN_ID'])
buffers = []
for fd in map(int, os.listdir('/proc/self/fd')):
    try:
        s = socket.socket(fileno=os.dup(fd))
    except OSError:
        continue
    if s.type == socket.SOCK_DGRAM and base <= s.getsockname()[1] < base + 250:
        buffers.append(s.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))
    s.close()
print('ready', *buffers, flush=True)
time.sleep(30)
'''
MIN_RECEIVE_BUFFER = 4 * 1024 * 1024


def udp_addresses(pid):
    """Local IPv4 addresses of the process's UDP sockets."""
    inodes = set()
    for fd in pathlib.Path(f'/proc/{pid}/fd').iterdir():
        target = os.readlink(fd)
        if target.startswith('socket:['):
            inodes.add(target[8:-1])
    addresses = set()
    for line in pathlib.Path(f'/proc/{pid}/net/udp').read_text().splitlines()[1:]:
        fields = line.split()
        if fields[9] in inodes:
            address = fields[1].split(':')[0]
            addresses.add(socket.inet_ntoa(struct.pack('<I', int(address, 16))))
    return addresses


@pytest.fixture
def participant():
    env = {**os.environ, 'ROS_LOCALHOST_ONLY': '0', 'FASTRTPS_DEFAULT_PROFILES_FILE': str(PROFILE),
           'ROS_DOMAIN_ID': str(100 + os.getpid() % 100)}
    process = subprocess.Popen([sys.executable, '-c', PARTICIPANT], env=env,
                               stdout=subprocess.PIPE, text=True)
    try:
        ready, *buffers = process.stdout.readline().split()
        assert ready == 'ready'
        time.sleep(1.0)
        yield process.pid, [int(b) for b in buffers]
    finally:
        process.kill()
        process.wait()


def test_the_profile_uses_no_shared_memory(participant):
    pid, _ = participant
    maps = pathlib.Path(f'/proc/{pid}/maps').read_text()
    assert '/dev/shm/fastrtps' not in maps


def test_the_profile_keeps_udp_on_loopback(participant):
    pid, _ = participant
    addresses = udp_addresses(pid)
    assert addresses, 'the participant opened no UDP sockets'
    # 239.255.0.1 is the discovery multicast group, joined on loopback only.
    assert addresses <= {'127.0.0.1', '239.255.0.1'}


def test_the_profile_asks_for_large_receive_buffers(participant):
    # The 212 KB kernel default overflowed on every node; the bridge's losses
    # turned into a heartbeat storm at 60% of a core (issue #17).
    _, buffers = participant
    assert buffers, 'the participant listens on no UDP port'
    assert min(buffers) >= MIN_RECEIVE_BUFFER, buffers


def docker_run_args():
    """The docker args file as run_dev.sh expands it: each line through eval echo."""
    args_file = WORKSPACE / 'docker' / '.isaac_ros_dev-dockerargs'
    script = 'readarray -t L < "$1"; for a in "${L[@]}"; do for w in $(eval "echo $a"); do echo "$w"; done; done'
    return subprocess.run(['bash', '-c', script, '_', str(args_file)],
                          capture_output=True, text=True, check=True).stdout.split()


def test_the_container_runs_with_the_profile_and_not_localhost_only():
    args = docker_run_args()
    pairs = list(zip(args, args[1:]))
    # run_dev.sh's echo swallows a bare -e, so the file must say --env.
    assert ('--env', 'ROS_LOCALHOST_ONLY=0') in pairs
    assert ('--env', f'FASTRTPS_DEFAULT_PROFILES_FILE={CONTAINER_PROFILE}') in pairs
