cmake_minimum_required(VERSION 3.16)
project(rc_hardware_control LANGUAGES CXX)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  pca9685_hardware_interface
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


install(
  DIRECTORY launch config description scripts
  DESTINATION share/rc_hardware_control
)

# Install Python scripts
install(
  PROGRAMS 
    scripts/checksync.py
    scripts/depth_check.py
    scripts/frame_rename.py
    scripts/goal_pose_relay.py
    scripts/led_off.py
    scripts/rs-imu-calibration.py
    scripts/stereo_sync_republisher.py
    scripts/test_ackermann.py
    scripts/test_bicycle.py
    scripts/test_steering_tracking.py
    scripts/vlm_brain.py
  DESTINATION lib/rc_hardware_control
)


## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
