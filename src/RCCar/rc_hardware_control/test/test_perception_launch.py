"""Perception bring-up: the three composable nodes live in one file and honour
the module's small interface (camera profile, obstacle band lower edge, robot
frame). Runs under `colcon test` in the container; needs launch and launch_ros.
"""
import importlib.util
import pathlib

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.utilities import evaluate_parameters

PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
LAUNCH_DIR = PACKAGE_DIR / 'launch'
PERCEPTION = LAUNCH_DIR / 'perception.launch.py'


def load_perception_module():
    spec = importlib.util.spec_from_file_location('perception_launch', PERCEPTION)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def params_of(node):
    (evaluated,) = evaluate_parameters(LaunchContext(), node.parameters)
    return evaluated


@pytest.fixture(scope='module')
def nodes():
    module = load_perception_module()
    camera, vslam, nvblox = module.perception_nodes(
        camera_profile='640x480x15',
        obstacle_band_lower_edge=0.07,
        robot_frame='base_footprint',
    )
    return {'camera': camera, 'vslam': vslam, 'nvblox': nvblox}


def test_camera_profile_reaches_both_stereo_streams(nodes):
    p = params_of(nodes['camera'])
    assert p['depth_module.profile'] == '640x480x15'
    assert p['depth_module.infra_profile'] == '640x480x15'


def test_obstacle_band_lower_edge_is_the_nvblox_slice_floor(nodes):
    p = params_of(nodes['nvblox'])
    assert p['static_mapper.esdf_slice_min_height'] == pytest.approx(0.07)
    assert isinstance(p['static_mapper.esdf_slice_min_height'], float)


def test_robot_frame_is_used_everywhere_the_stack_reads_it(nodes):
    assert params_of(nodes['vslam'])['base_frame'] == 'base_footprint'
    nvblox = params_of(nodes['nvblox'])
    assert nvblox['pose_frame'] == 'base_footprint'
    assert nvblox['map_clearing_frame_id'] == 'base_footprint'


def test_imu_fusion_is_pinned_off(nodes):
    assert params_of(nodes['vslam'])['enable_imu_fusion'] is False
    camera = params_of(nodes['camera'])
    assert camera['enable_gyro'] is False
    assert camera['enable_accel'] is False


def test_nvblox_maps_in_odom_per_adr_0001(nodes):
    assert params_of(nodes['nvblox'])['global_frame'] == 'odom'


def test_launch_declares_the_three_interface_arguments():
    module = load_perception_module()
    declared = {
        e.name for e in module.generate_launch_description().entities
        if isinstance(e, DeclareLaunchArgument)
    }
    assert declared == {'camera_profile', 'obstacle_band_lower_edge', 'robot_frame'}


def test_composable_nodes_are_defined_in_exactly_one_file():
    defining = sorted(
        f.name for f in LAUNCH_DIR.glob('*.launch.py')
        if 'ComposableNode(' in f.read_text()
    )
    assert defining == ['perception.launch.py']


def test_launch_directory_holds_only_the_three_launches():
    names = sorted(f.name for f in LAUNCH_DIR.glob('*.launch.py'))
    assert names == ['perception.launch.py', 'perception_only.launch.py', 'rccarauto.launch.py']


def test_no_launch_uses_map_as_a_global_frame():
    for f in LAUNCH_DIR.glob('*.launch.py'):
        text = f.read_text()
        assert "'global_frame': 'map'" not in text, f.name
        assert '"global_frame": "map"' not in text, f.name
