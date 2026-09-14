"""Vehicle geometry: the measured dimensions in the URDF xacro properties.

The xacro file is the single source. This reads its top-level
<xacro:property name=... value=...> elements and evaluates the ${...}
expressions among them, so tests can assert every derived value (Steering
controller wheelbase and radii, costmap footprint, turning radius floor,
Arrival tolerance, obstacle band lower edge) against the one place the
numbers are typed. No xacro dependency: properties here are plain numbers
and arithmetic over other properties.
"""
import math
import pathlib
import re
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple

XACRO_NS = 'http://www.ros.org/wiki/xacro'


def _package_root() -> pathlib.Path:
    """The source tree when run from it, else the installed share directory."""
    source = pathlib.Path(__file__).resolve().parents[1]
    if (source / 'description' / 'description.urdf.xacro').exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return pathlib.Path(get_package_share_directory('rc_hardware_control'))


DEFAULT_XACRO = _package_root() / 'description' / 'description.urdf.xacro'

# nvblox obstacle band, in nvblox's global frame (odom; z is zero where visual
# SLAM started, which is the floor on level ground). nvblox floors each edge to
# a voxel row, takes rows inclusively, and marks a voxel as an obstacle when its
# TSDF distance is within one voxel of a surface. The row touching the floor
# therefore always registers the floor, so the lower edge must fall in row one
# or higher; the row's centre minus one voxel is the lowest object top that
# registers. The upper edge is the spec's 50 cm: above that the camera looks
# over it and the car drives under it.
NVBLOX_VOXEL_SIZE = 0.05
OBSTACLE_BAND_LOWER_EDGE = 0.06
OBSTACLE_BAND_UPPER_EDGE = 0.50

_EXPR = re.compile(r'^\$\{(.*)\}$')
_SAFE_NAMES = {name: getattr(math, name) for name in ('sqrt', 'atan', 'tan', 'sin', 'cos', 'pi')}


def load_properties(xacro_path: pathlib.Path = DEFAULT_XACRO) -> Dict[str, float]:
    """Every top-level xacro property as a float, expressions evaluated in file order."""
    root = ET.parse(xacro_path).getroot()
    values: Dict[str, float] = {}
    for prop in root.findall(f'{{{XACRO_NS}}}property'):
        name, raw = prop.get('name'), prop.get('value')
        match = _EXPR.match(raw.strip())
        expr = match.group(1) if match else raw
        values[name] = float(eval(expr, {'__builtins__': {}}, {**_SAFE_NAMES, **values}))
    return values


def footprint(props: Dict[str, float], padding_m: float = 0.02) -> List[Tuple[float, float]]:
    """Nav2 footprint polygon in base_footprint: the chassis box plus padding
    all round. The box is centred on base_footprint, so the polygon is too."""
    half_length = props['chassis_length'] / 2 + padding_m
    half_width = props['chassis_width'] / 2 + padding_m
    return [
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, -half_width),
        (-half_length, half_width),
    ]


def minimum_turning_radius_floor(props: Dict[str, float]) -> float:
    """Physical minimum turning radius of base_footprint, which sits midway
    between the axles: the rear axle midpoint turns at wheelbase / tan(lock)."""
    rear_axle_radius = props['wheelbase'] / math.tan(props['steer_max_angle'])
    return math.hypot(rear_axle_radius, props['wheelbase'] / 2)


def arrival_tolerance(props: Dict[str, float]) -> float:
    """Arrival is being within about one car length of the Goal."""
    return props['chassis_length']


def obstacle_band_first_row(lower_edge: float = OBSTACLE_BAND_LOWER_EDGE,
                            voxel: float = NVBLOX_VOXEL_SIZE) -> int:
    """Index of the lowest voxel row nvblox includes; row 0 touches the floor."""
    return math.floor(lower_edge / voxel)


def obstacle_detection_threshold(lower_edge: float = OBSTACLE_BAND_LOWER_EDGE,
                                 voxel: float = NVBLOX_VOXEL_SIZE) -> float:
    """Lowest object top, in metres above the floor, that registers as an obstacle."""
    row_centre = (obstacle_band_first_row(lower_edge, voxel) + 0.5) * voxel
    return row_centre - voxel
