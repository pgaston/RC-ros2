"""Vehicle geometry is owned once, by the xacro properties. Every value the
stack derives from it must match. Pure Python plus PyYAML; runs on the host.
"""
import pathlib

import pytest
import yaml

from rc_hardware_control import vehicle_geometry as vg

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / 'config'


@pytest.fixture(scope='module')
def props():
    return vg.load_properties()


@pytest.fixture(scope='module')
def steering():
    doc = yaml.safe_load((CONFIG / 'steer_bot_hardware.yaml').read_text())
    return doc['bicycle_steering_controller']['ros__parameters']


@pytest.fixture(scope='module')
def nav2():
    return yaml.safe_load((CONFIG / 'my_custom_nav2_params.yaml').read_text())


def footprint_of(costmap_params):
    return [tuple(p) for p in yaml.safe_load(costmap_params['footprint'])]


def test_measured_properties_are_present(props):
    for name in ('wheel_radius', 'wheelbase', 'track_width', 'chassis_length',
                 'chassis_width', 'ground_clearance', 'steer_max_angle'):
        assert props[name] > 0, name


def test_steering_controller_uses_the_measured_wheel(props, steering):
    assert steering['wheelbase'] == pytest.approx(props['wheelbase'])
    assert steering['front_wheel_radius'] == pytest.approx(props['wheel_radius'])
    assert steering['rear_wheel_radius'] == pytest.approx(props['wheel_radius'])


def test_both_costmaps_use_the_chassis_footprint(props, nav2):
    expected = [(pytest.approx(x), pytest.approx(y)) for x, y in vg.footprint(props)]
    local = nav2['local_costmap']['local_costmap']['ros__parameters']
    global_ = nav2['global_costmap']['global_costmap']['ros__parameters']
    assert footprint_of(local) == expected
    assert footprint_of(global_) == expected


def test_planner_turning_radius_respects_the_physical_floor(props, nav2):
    planner = nav2['planner_server']['ros__parameters']['GridBased']
    assert planner['minimum_turning_radius'] >= vg.minimum_turning_radius_floor(props)


def test_physical_turning_radius_floor_matches_the_measured_circle(props):
    # 25 in inside diameter at the inner rear tyre, rear-axle midpoint at 0.45 m
    assert vg.minimum_turning_radius_floor(props) == pytest.approx(0.47, abs=0.01)


def test_arrival_tolerance_is_one_car_length(props, nav2):
    checker = nav2['controller_server']['ros__parameters']['general_goal_checker']
    assert checker['xy_goal_tolerance'] == pytest.approx(vg.arrival_tolerance(props))


def test_obstacle_band_excludes_the_floor_voxel_row():
    # Row 0 touches the floor and would register it everywhere.
    assert vg.obstacle_band_first_row() >= 1


def test_obstacle_band_catches_anything_the_chassis_cannot_clear(props):
    # An object taller than the ground clearance hits the chassis, so the lowest
    # object top that registers must be no higher than the clearance.
    assert vg.obstacle_detection_threshold() <= props['ground_clearance']


def test_obstacle_band_upper_edge_is_above_the_lower(props):
    assert vg.OBSTACLE_BAND_UPPER_EDGE > vg.OBSTACLE_BAND_LOWER_EDGE
