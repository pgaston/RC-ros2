# RC Car

An autonomous RC car on a Jetson that maps its surroundings, indoors and on patio or grass, with a depth camera and drives itself to a target chosen from a natural-language instruction.

## Language

### Vehicle

**Traction joint**:
The ESC-driven joint that moves the car forward or backward. Commanded by velocity.
_Avoid_: Tracking joint, drive joint, throttle

**Steering joint**:
The servo-driven joint that sets the front-wheel angle. Commanded by position.
_Avoid_: Turn joint

### Navigation

**Goal**:
The pose the car is currently driving to. Only position matters; heading is ignored.
_Avoid_: Target, waypoint, destination, goal pose

**Target**:
A thing in the world, identified from a natural-language instruction, that resolves to a Goal.
_Avoid_: Goal, object, landmark

**Arrival**:
The state of being within roughly one car length of the Goal, regardless of heading.
_Avoid_: Reached, success, done

**Stuck**:
The state of making no progress toward the Goal for a bounded time.
_Avoid_: Blocked, failed, oscillating

**Recovery**:
A short straight reverse taken when Stuck. The only situation in which the car reverses.
_Avoid_: Backup, escape, retry

### Control layers

**Path follower**:
The Nav2 plugin that turns a planned path into a velocity command, honouring the car's minimum turning radius.
_Avoid_: Controller, local planner, Ackermann controller

**Steering controller**:
The ros2_control module that turns a velocity command into a steering angle and a traction speed using the bicycle model.
_Avoid_: Controller, bicycle controller, Ackermann controller

**Car-like**:
Motion with a minimum turning radius and no turning in place. The kinematic constraint both control layers must respect.
_Avoid_: Ackermann, bicycle, non-holonomic

### Modules

**Goal relay**:
The only NavigateToPose client. Owns Goal admission, preemption, and outcome, and publishes accepted, rejected, arrived, stuck, and aborted on a status topic. Admits a Goal only when the robot pose, an occupancy grid, and a healthy Perception watchdog status are all present. Anything that wants the car to go somewhere, including a future VLM brain, calls it.
_Avoid_: Goal server, nav client, brain

**Perception bring-up**:
The launch module that starts the depth camera, visual SLAM, and nvblox together. Its interface is the camera profile, the obstacle band lower edge, and the robot frame. Run by the Perception watchdog in the full launch, and included by a camera-only launch.
_Avoid_: Sensor stack, camera launch

**Vehicle geometry**:
The measured dimensions in the URDF xacro properties, the single source for every derived value: controller wheel radius and wheelbase, costmap footprint, minimum turning radius floor, Arrival tolerance, and the obstacle band lower edge. A test asserts the derived values match.
_Avoid_: Robot params, dimensions config

**Velocity mux**:
The only publisher the Steering controller listens to. Merges the teleop source, the Perception watchdog's hold, and Nav2's command by priority: teleop always wins, the hold outranks Nav2, a source that goes silent for longer than its timeout drops out, and with nothing fresh it publishes zero so the car stops. Its interface is the source topics, priorities and timeouts in its configuration.
_Avoid_: Twist mux, cmd_vel mux, arbiter, deadman node

**Perception watchdog**:
The owner of the Perception bring-up in the full launch. Runs it as a child process, holds the car through the Velocity mux while depth, visual SLAM odometry or the occupancy grid is stale, and when a stream stalls or the bring-up exits it cancels the Goal and restarts the bring-up. Publishes starting, healthy, stale, restarting, and down on a status topic.
_Avoid_: Camera watchdog, supervisor, heartbeat, perception monitor

**Hardware interface**:
The ros2_control plugin that turns a Steering joint angle and a Traction joint wheel speed into servo and ESC pulses. Its interface is the ros2_control block in the URDF; every parameter there is read and nothing else configures it. It has no feedback; the Steering controller runs open loop.
_Avoid_: Motor driver, PCA9685 node, ESC controller
