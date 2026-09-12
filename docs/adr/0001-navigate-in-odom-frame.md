---
status: accepted
---

# Navigate in the odom frame, not the map frame

Visual SLAM publishes both a map frame, corrected by loop closure, and an odom frame that drifts smoothly. Nav2's default is to plan in map. We plan, hold Goals, and build the costmaps in odom instead, because a loop-closure correction moves the car's pose under a car-like path follower mid-drive and produces exactly the oscillation and wandering we set out to remove. Odom drift over a patio-length drive stays well inside the Arrival tolerance.

## Consequences

- A Goal is a pose in odom and slowly drifts from the physical spot that was clicked. Acceptable at the distances a patio or a room allows.
- Nothing persists between runs. There is no saved map, and a Goal from a previous run is meaningless.
- When the VLM step resolves a Target to a Goal, it must do so in odom at the moment of resolution. Revisit this decision if Targets must survive long drives or a restart.
