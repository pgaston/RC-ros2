"""Full launch: what Foxglove can reach over the car's wifi link (issues #10, #16).
Runs under `colcon test` in the container; needs launch, launch_ros and nav2_common.
"""
import importlib.util
import pathlib
import re

import pytest
from launch import LaunchContext
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

LAUNCH_DIR = pathlib.Path(__file__).resolve().parents[1] / 'launch'
PREVIEW_TOPIC = '/camera_preview/image/compressed'


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def nodes():
    description = load('rccarauto.launch.py').generate_launch_description()
    return {n.node_executable: n for n in description.entities if isinstance(n, Node)}


def params_of(node):
    # Humble's Node keeps its parameters private.
    (evaluated,) = evaluate_parameters(LaunchContext(), node._Node__parameters)
    return evaluated


def bridge_passes(bridge, topic):
    # foxglove_bridge whole-matches each pattern (std::regex_match).
    return any(re.fullmatch(p, topic) for p in params_of(bridge)['topic_whitelist'])


def test_the_preview_reads_the_emitter_off_infra_stream(nodes):
    preview = params_of(nodes['camera_preview.py'])
    assert preview['input_topic'] == load('perception.launch.py').EMITTER_OFF_INFRA1
    assert preview['output_topic'] == PREVIEW_TOPIC
    assert preview['rate_hz'] == 5.0
    assert preview['max_width'] == 424


def test_the_bridge_passes_the_preview_and_no_full_rate_image(nodes):
    bridge = nodes['foxglove_bridge']
    assert bridge_passes(bridge, PREVIEW_TOPIC)
    for full_rate in ('/camera/infra1/image_rect_raw/compressed',
                      '/camera/depth/image_rect_raw/compressedDepth',
                      '/depth/image_rect_raw',
                      '/emitter_off/infra1/image_rect_raw'):
        assert not bridge_passes(bridge, full_rate), full_rate


def test_the_bridge_drops_messages_past_one_megabyte_queued(nodes):
    assert params_of(nodes['foxglove_bridge'])['send_buffer_limit'] == 1000000


def test_the_bridge_still_passes_what_driving_needs(nodes):
    bridge = nodes['foxglove_bridge']
    for topic in ('/tf', '/tf_static', '/plan', '/clicked_point', '/goal_relay/status',
                  '/cmd_vel_teleop', '/local_costmap/costmap', '/global_costmap/costmap',
                  '/perception/status'):
        assert bridge_passes(bridge, topic), topic


def test_the_bridge_takes_tf_best_effort_and_tf_static_reliably(nodes):
    # A reliable /tf reader in the bridge that stalls makes both /tf writers
    # resend to it and heartbeat every /tf reader, ~85k packets a second
    # (issue #17). /tf_static is latched and rare, so it stays reliable.
    patterns = params_of(nodes['foxglove_bridge'])['best_effort_qos_topic_whitelist']
    assert any(re.fullmatch(p, '/tf') for p in patterns)
    assert not any(re.fullmatch(p, '/tf_static') for p in patterns)
