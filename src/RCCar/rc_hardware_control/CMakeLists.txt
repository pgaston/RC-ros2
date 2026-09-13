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

# Install Python scripts as executables so `ros2 run` and the launch find them
install(
  PROGRAMS
    scripts/cmd_vel_logger.py
    scripts/frame_rename.py
    scripts/goal_pose_relay.py
    scripts/test_bicycle.py
  DESTINATION lib/rc_hardware_control
)


if(BUILD_TESTING)
  # Perception launch interface; needs launch and launch_ros, no hardware.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_perception_launch test/test_perception_launch.py)
endif()

## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
