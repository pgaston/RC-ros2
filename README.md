
# RC car autonomous control

Features:
- ROS2 true, as much as possible.   Note this is using Humble.   While the most recent update from nVidia is moving to Jazzy.    This change alone will break things.
- leverage nVidia stack as much as possible (via the NVidia docker setup for Jetson (Orin) Nano/x86)
- WebRTC

![general architecture](image.png)

![stream robot video to user](image-1.png)

[isaac ROS base page](https://nvidia-isaac-ros.github.io/v/release-3.2/index.html)


Summary Table: Component MappingGoalIsaac ROS PackageHardware Resource UsedVIO / Odometryisaac_ros_visual_slamGPU & IMUStereo Depthisaac_ros_ess (Optional)DLA / GPUMappingisaac_ros_nvbloxGPU (CUDA)PlanningNav2 (Integrated)CPU / GPUWebRTC Videoisaac_ros_h264_encoderNVENC (Video Encoder)Remote Control10isaac_ros_mission_client11CPU12

Commands of high interest:

```
Launch - note the-d flag

cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts
./run_dev.sh -d ${ISAAC_ROS_WS}

Testing the hardware control
colcon build --packages-select rc_hardware_control

ros2 launch rc_hardware_control steering_tracking_example.launch.py

test
ros2 topic pub /bicycle_steering_controller/reference_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.01}, angular: {z: 0.0}}" 

and third window
ros2 topic echo /joint_states
ros2 topic echo /joint_states --field velocity


-or- only for joint, steer, traction
ros2 launch rc_hardware_control basic_steering_traction.launch.py


Start with everything stopped/centered
ros2 topic pub /steering_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]" --once
ros2 topic pub /traction_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]" --once



```

Also
```
i2c access - fixed (using gemini AI - go there to see how)
sudo chmod 666 /dev/gpiochip0 /dev/gpiochip1 /dev/i2c-0 /dev/i2c-1 /dev/i2c-7

need video (only needed if startted without a video monitor live ???)
sudo rm /dev/fb0 && sudo mknod /dev/fb0 c 29 0 && sudo chmod 666 /dev/fb0

```

## Github sync
Our repositories so far under this are pca9685_ros2_control and TeleOpROS2 (which will go away/change)
```
Push

go to folder
git add .
git commit -m "next commit?"
git push

```

## vcs
The standard "Sync Up" workflow for a vcstool workspace looks like this:

- Check Status: vcs status src (See what changed).
- Commit Locally: Go into individual src/repo_name folders and git commit.
- Push Sub-Repos: vcs custom src --args push (Push the code).
- Update Map: vcs export src --exact > workspace.repos (Capture new versions).
- Push Map: git push at the top level (Push the workspace configuration).



# t.b.d.

