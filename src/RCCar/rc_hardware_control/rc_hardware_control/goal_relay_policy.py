"""The goal relay's rules: when a Goal is admitted, which way it faces, and what its end is called.

Pure Python, no ROS. The node gathers the facts (is Nav2 there, is the robot
pose known, has an occupancy grid arrived, what does the perception watchdog
say) and asks for a refusal reason; it asks for the Goal heading from the
robot and Goal positions; and when Nav2 reports a result it asks for the
status value and reason to publish.

Status values: accepted, rejected, arrived, stuck, aborted.
"""
import math
from dataclasses import dataclass
from typing import Optional, Tuple

# action_msgs/msg/GoalStatus
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

# Closer than this the bearing to the Goal is noise; keep the car's heading.
MIN_BEARING_DISTANCE_M = 0.05


@dataclass(frozen=True)
class Readiness:
    server_available: bool      # Nav2's navigate_to_pose action server is up
    robot_pose_known: bool      # a recent transform from the global frame to the robot frame exists
    grid_received: bool         # at least one occupancy grid has arrived
    perception_watched: bool    # the perception watchdog's status topic is configured
    perception_status: Optional[str]   # its latest status, None until one arrives


def refusal(r: Readiness, action: str, global_frame: str, robot_frame: str) -> Optional[str]:
    """Why a Goal cannot be admitted now, or None when it can."""
    if not r.server_available:
        return f'{action} is not available'
    if not r.robot_pose_known:
        return f'no recent transform from {global_frame} to {robot_frame}'
    if not r.grid_received:
        return 'no occupancy grid received yet'
    if r.perception_watched:
        if r.perception_status is None:
            return 'no perception status yet'
        if r.perception_status != 'healthy':
            return f'perception {r.perception_status}'
    return None


def yaw_of(x: float, y: float, z: float, w: float) -> float:
    """Heading about +z of a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_of_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation by yaw about +z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def goal_heading(robot_x: float, robot_y: float, robot_yaw: float,
                 goal_x: float, goal_y: float) -> float:
    """The bearing from the car to the Goal.

    Smac Hybrid-A* plans to the Goal's heading even though the goal checker
    ignores it; facing along the bearing ends the path pointing the way the
    car was already going, instead of adding a loop to face a fixed axis.
    """
    dx, dy = goal_x - robot_x, goal_y - robot_y
    if math.hypot(dx, dy) < MIN_BEARING_DISTANCE_M:
        return robot_yaw
    return math.atan2(dy, dx)


def outcome(goal_status: int, recoveries: int, plan_received: bool,
            perception_status: Optional[str]) -> Tuple[str, str]:
    """(value, reason) for a Goal that Nav2 finished and nothing replaced.

    recoveries is Nav2's number_of_recoveries feedback, which counts recovery
    behaviours ticked (each Recovery here ticks Wait, BackUp and Wait), so
    only zero or not zero means anything. plan_received says whether the
    planner published a path after the Goal was sent. perception_status is
    None when the watchdog is not watched.
    """
    if goal_status == STATUS_SUCCEEDED:
        return 'arrived', 'within the Arrival tolerance'
    if goal_status not in (STATUS_CANCELED, STATUS_ABORTED):
        return 'aborted', f'unexpected goal status {goal_status}'
    if perception_status is not None and perception_status != 'healthy':
        return 'aborted', f'perception lost ({perception_status})'
    if goal_status == STATUS_CANCELED:
        return 'aborted', 'cancelled outside the goal relay'
    if recoveries > 0:
        return 'stuck', 'Recoveries did not free the car'
    if not plan_received:
        return 'rejected', 'no path to the Goal'
    return 'aborted', 'Nav2 aborted the Goal'
