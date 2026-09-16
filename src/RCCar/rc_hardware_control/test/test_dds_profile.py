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
import time, rclpy
from std_msgs.msg import String
rclpy.init()
node = rclpy.create_node('dds_profile_probe')
node.create_publisher(String, '/dds_profile_probe', 10)
print('ready', flush=True)
time.sleep(30)
'''


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
        assert process.stdout.readline().strip() == 'ready'
        time.sleep(1.0)
        yield process.pid
    finally:
        process.kill()
        process.wait()


def test_the_profile_uses_no_shared_memory(participant):
    maps = pathlib.Path(f'/proc/{participant}/maps').read_text()
    assert '/dev/shm/fastrtps' not in maps


def test_the_profile_keeps_udp_on_loopback(participant):
    addresses = udp_addresses(participant)
    assert addresses, 'the participant opened no UDP sockets'
    # 239.255.0.1 is the discovery multicast group, joined on loopback only.
    assert addresses <= {'127.0.0.1', '239.255.0.1'}


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
