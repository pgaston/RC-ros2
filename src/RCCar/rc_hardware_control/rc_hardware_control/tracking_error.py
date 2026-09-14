"""How far the car strays from Nav2's plan, per Goal (issue #7).

Pure Python, no ROS: the tracking recorder feeds in each plan Nav2 publishes
and each car position from odometry, both in odom, and gets back the
cross-track error, the distance from the car to the nearest point of the
path. When the Goal ends it asks for a summary.

Two errors are kept. Against the latest plan: Nav2 replans at 1 Hz from
wherever the car is, so this shows how well the Path follower tracks the path
it is given, and it is what #7's "within half a car width of the plan" is
judged on. Against the Goal's first plan: this shows how far the drive strayed
from the route first chosen, which a replan can hide.
"""
import math
from dataclasses import astuple, dataclass, fields
from typing import List, Optional, Sequence, Tuple

Point = Tuple[float, float]


def distance_to_path(x: float, y: float, path: Sequence[Point]) -> float:
    """Distance from (x, y) to the polyline through path; inf for an empty path."""
    if not path:
        return math.inf
    if len(path) == 1:
        return math.hypot(x - path[0][0], y - path[0][1])
    best = math.inf
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        t = 0.0 if length_sq == 0.0 else min(1.0, max(0.0, ((x - ax) * dx + (y - ay) * dy) / length_sq))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def percentile(values: Sequence[float], q: float) -> float:
    """Nearest-rank percentile, q from 0 to 100. NaN for no values."""
    if not values:
        return math.nan
    ordered = sorted(values)
    return ordered[max(1, math.ceil(q / 100.0 * len(ordered))) - 1]


@dataclass(frozen=True)
class GoalSummary:
    goal: int                       # 1 for the first Goal the recorder saw
    outcome: str                    # the relay status that ended it
    duration_s: float
    travelled_m: float
    samples: int                    # positions measured against a plan
    max_error_m: float              # against the latest plan
    mean_error_m: float
    p95_error_m: float
    within_limit: float             # fraction of samples within limit_m of the latest plan
    max_error_first_plan_m: float   # against the Goal's first plan
    limit_m: float                  # half a car width

    @classmethod
    def header(cls) -> List[str]:
        return [f.name for f in fields(cls)]

    def row(self) -> List[str]:
        return [f'{v:.4f}' if isinstance(v, float) else str(v) for v in astuple(self)]

    def describe(self) -> str:
        if not self.samples:
            return f'Goal {self.goal} {self.outcome}: no position measured against a plan'
        return (f'Goal {self.goal} {self.outcome}: {self.duration_s:.1f} s, {self.travelled_m:.2f} m, '
                f'cross-track max {self.max_error_m:.2f} m, p95 {self.p95_error_m:.2f} m, '
                f'{self.within_limit:.0%} within {self.limit_m:.2f} m; '
                f'max {self.max_error_first_plan_m:.2f} m from the first plan')


class GoalTrack:
    """One Goal, from the relay's accepted to its final status."""

    def __init__(self, goal: int, started_at: float, limit_m: float):
        self.goal = goal
        self._started_at = started_at
        self._limit_m = limit_m
        self._first_plan: Optional[List[Point]] = None
        self._plan: Optional[List[Point]] = None
        self._errors: List[float] = []
        self._first_plan_errors: List[float] = []
        self._last: Optional[Point] = None
        self._travelled = 0.0

    def plan(self, points: Sequence[Point]) -> None:
        """A plan Nav2 published for this Goal. An empty plan is ignored."""
        if not points:
            return
        self._plan = list(points)
        if self._first_plan is None:
            self._first_plan = self._plan

    def position(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """A car position: (error against the latest plan, against the first), or None before any plan."""
        if self._last is not None:
            self._travelled += math.hypot(x - self._last[0], y - self._last[1])
        self._last = (x, y)
        if self._plan is None:
            return None
        errors = distance_to_path(x, y, self._plan), distance_to_path(x, y, self._first_plan)
        self._errors.append(errors[0])
        self._first_plan_errors.append(errors[1])
        return errors

    def finish(self, ended_at: float, outcome: str) -> GoalSummary:
        e = self._errors
        return GoalSummary(
            goal=self.goal,
            outcome=outcome,
            duration_s=ended_at - self._started_at,
            travelled_m=self._travelled,
            samples=len(e),
            max_error_m=max(e) if e else math.nan,
            mean_error_m=sum(e) / len(e) if e else math.nan,
            p95_error_m=percentile(e, 95),
            within_limit=sum(1 for v in e if v <= self._limit_m) / len(e) if e else math.nan,
            max_error_first_plan_m=max(self._first_plan_errors) if e else math.nan,
            limit_m=self._limit_m,
        )
