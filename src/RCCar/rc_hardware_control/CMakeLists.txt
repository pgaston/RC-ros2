cmake_minimum_required(VERSION 3.16)
project(rc_hardware_control LANGUAGES CXX)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  pca9685_hardware_interface
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


install(
  DIRECTORY launch config description scripts
  DESTINATION share/rc_hardware_control
)

# Pure Python shared by the scripts and the tests (rc_hardware_control/)
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts as executables so `ros2 run` and the launch find them
install(
  PROGRAMS
    scripts/cmd_vel_logger.py
    scripts/frame_rename.py
    scripts/goal_pose_relay.py
    scripts/test_bicycle.py
    scripts/velocity_mux.py
  DESTINATION lib/rc_hardware_control
)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Perception launch interface; needs launch and launch_ros, no hardware.
  ament_add_pytest_test(test_perception_launch test/test_perception_launch.py)
  # Velocity mux selection policy; pure Python.
  ament_add_pytest_test(test_velocity_mux_policy test/test_velocity_mux_policy.py)
endif()

## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
