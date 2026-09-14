# realsense_splitter (vendored)

From NVIDIA-ISAAC-ROS/isaac_ros_nvblox, branch release-3.2, commit 7908a18,
directory nvblox_examples/realsense_splitter. Apache-2.0 (LICENSE here).

Not in the Isaac ROS image; built in this workspace for issue #15. Upstream
ships it with a COLCON_IGNORE, removed here. The code is otherwise unchanged.

It republishes the RealSense infra images from frames with the IR emitter off
(for visual SLAM) and depth from frames with it on (for nvblox), reading
frame_emitter_mode from each frame's metadata. The camera must run with
depth_module.emitter_on_off true. Wiring: launch/perception.launch.py in
rc_hardware_control.
