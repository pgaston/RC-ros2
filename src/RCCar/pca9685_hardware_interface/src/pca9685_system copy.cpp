#include "pca9685_hardware_interface/pca9685_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"



namespace pca9685_hardware_interface
{
hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{

  pca.set_pwm_freq(50.0);

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  joint_configs_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];
    
    // Pca9685System has one command interface on each output
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Check if it's position, velocity, or effort interface
    const std::string& interface_name = joint.command_interfaces[0].name;
    if (interface_name != hardware_interface::HW_IF_VELOCITY && 
        interface_name != hardware_interface::HW_IF_POSITION &&
        interface_name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has '%s' command interface. Only 'position', 'velocity', or 'effort' expected.", 
        joint.name.c_str(), interface_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Parse joint parameters
    joint_configs_[i].interface_type = interface_name;
    
    // Get channel parameter (required)
    auto channel_param = joint.parameters.find("channel");
    if (channel_param == joint.parameters.end())
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' missing required 'channel' parameter.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_configs_[i].channel = std::stoi(channel_param->second);

    // Get servo parameters for position joints
    if (interface_name == hardware_interface::HW_IF_POSITION)
    {
      auto min_angle = joint.parameters.find("min_angle");
      if (min_angle != joint.parameters.end()) {
        joint_configs_[i].min_angle = std::stod(min_angle->second);
      }
      
      auto max_angle = joint.parameters.find("max_angle");
      if (max_angle != joint.parameters.end()) {
        joint_configs_[i].max_angle = std::stod(max_angle->second);
      }
      
      auto min_pulse = joint.parameters.find("min_pulse_us");
      if (min_pulse != joint.parameters.end()) {
        joint_configs_[i].min_pulse_us = std::stoi(min_pulse->second);
      }
      
      auto max_pulse = joint.parameters.find("max_pulse_us");
      if (max_pulse != joint.parameters.end()) {
        joint_configs_[i].max_pulse_us = std::stoi(max_pulse->second);
      }
    }
    // Effort interface (LED PWM) - no additional parameters needed

    RCLCPP_INFO(
      rclcpp::get_logger("Pca9685SystemHardware"),
      "Joint '%s' configured: channel=%d, type=%s", 
      joint.name.c_str(), joint_configs_[i].channel, interface_name.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const std::string& interface_type = joint_configs_[i].interface_type;
    
    // Always export position and velocity for Ackermann compatibility
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    
    // Also export effort state interface for effort-controlled joints (LEDs)
    if (interface_type == hardware_interface::HW_IF_EFFORT)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_[i]));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    // Export command interface matching the configured type
    const std::string& interface_type = joint_configs_[i].interface_type;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, interface_type, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[i] = 0;
    }
    // If this joint is the traction ESC (velocity interface), send neutral PWM (1.5 ms) for arming
    const JointConfig& config = joint_configs_[i];
    if (config.interface_type == hardware_interface::HW_IF_VELOCITY) {
      pca.set_pwm_ms(config.channel, 1.5); // 1.5 ms = neutral
      RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"),
        "ESC on channel %d initialized with neutral PWM (1.5 ms) for arming.", config.channel);
    }
  }

  // Set arming end time to 2 seconds from now
  esc_arming_end_time_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);

  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Update state values based on command interface type
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (!std::isnan(hw_commands_[i]))
    {
      const JointConfig& config = joint_configs_[i];
      
      if (config.interface_type == hardware_interface::HW_IF_POSITION)
      {
        // For position control, position state = command, velocity = 0
        hw_positions_[i] = hw_commands_[i];
        hw_velocities_[i] = 0.0;
      }
      else if (config.interface_type == hardware_interface::HW_IF_VELOCITY)
      {
        // For velocity control, integrate velocity to get position
        double dt = period.seconds();
        hw_positions_[i] += hw_commands_[i] * dt;
        hw_velocities_[i] = hw_commands_[i];
      }
      else if (config.interface_type == hardware_interface::HW_IF_EFFORT)
      {
        // For effort control (LEDs), effort state = command
        // Position and velocity states remain at 0
        hw_positions_[i] = 0.0;
        hw_velocities_[i] = 0.0;
        // Effort state is directly linked to command in export_state_interfaces
      }
    }
  }

  return hardware_interface::return_type::OK;
}

double Pca9685SystemHardware::command_to_duty_cycle_velocity(double command)
{
  // For velocity commands (ESC control) - adjusted for motor deadband
  // RC ESCs typically need ~0.3-0.4 minimum command to start moving
  // Scale so that small linear.x values give usable motor commands
  
  double scale_factor = 0.15;  // Increased scaling: linear.x=0.1 → motor=0.43
  double scaled_command = command * scale_factor;
  
  // Apply minimum threshold to overcome ESC deadband
  double min_threshold = 0.35;  // Minimum command to move motor
  if (std::abs(scaled_command) > 0.02 && std::abs(scaled_command) < min_threshold) {
    scaled_command = (scaled_command > 0) ? min_threshold : -min_threshold;
  }
  
  double min_input = -1.0;
  double max_input = 1.0;

  double clamped_command = std::clamp(scaled_command, min_input, max_input);

  // Use same pulse width range as working PCA9685ServoESC.py:
  double min_duty_cycle = 1.0;  // 1ms pulse (full reverse)
  double neutral_duty_cycle = 1.5; // 1.5ms pulse (stop)  
  double max_duty_cycle = 2.0;  // 2ms pulse (full forward)

  if (std::abs(clamped_command) < 0.02) {  // Deadband to ensure clean stop
    return neutral_duty_cycle;  
  } else if (clamped_command > 0.0) {
    // Forward: interpolate between neutral and max
    return neutral_duty_cycle + clamped_command * (max_duty_cycle - neutral_duty_cycle);
  } else {
    // Reverse: interpolate between neutral and min  
    return neutral_duty_cycle + clamped_command * (neutral_duty_cycle - min_duty_cycle);
  }
}

double Pca9685SystemHardware::command_to_duty_cycle_position(double command, const JointConfig& config)
{
  // For position commands (servo control)
  // Command and min/max_angle are in radians
  double angle = std::clamp(command, config.min_angle, config.max_angle);
  // Convert to pulse width
  double pulse_width_us = angle_to_pulse_width(angle, config);
  // Convert microseconds to milliseconds for PCA9685
  return pulse_width_us / 1000.0;
}

double Pca9685SystemHardware::command_to_duty_cycle_effort(double command)
{
  // For effort commands (LED PWM control)
  // Command is 0.0 to 1.0 representing 0% to 100% brightness
  double clamped_command = std::clamp(command, 0.0, 1.0);
  
  // Convert to duty cycle in milliseconds (0-20ms for 50Hz PWM)
  // 0% = 0ms, 100% = 20ms
  return clamped_command * 20.0;
}

double Pca9685SystemHardware::angle_to_pulse_width(double angle, const JointConfig& config)
{
  // Linear interpolation between min and max pulse widths, all in radians
  // for some reason this must be reversed  
  double normalized_angle = (config.max_angle - angle) / (config.max_angle - config.min_angle);
  // double normalized_angle = (angle - config.min_angle) / (config.max_angle - config.min_angle);
  return config.min_pulse_us + normalized_angle * (config.max_pulse_us - config.min_pulse_us);
}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto now = std::chrono::steady_clock::now();
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_commands_[i]))
    {
      continue;  // Skip uninitialized commands
    }

    double duty_cycle;
    const JointConfig& config = joint_configs_[i];

    // During arming period, always send neutral to ESC (velocity interface)
    if (config.interface_type == hardware_interface::HW_IF_VELOCITY && now < esc_arming_end_time_)
    {
      duty_cycle = 1.5;
      pca.set_pwm_ms(config.channel, duty_cycle);
      RCLCPP_DEBUG(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "[ESC ARMING] Joint %u (%s): forced neutral duty_cycle=%.3f, channel=%d",
        i, config.interface_type.c_str(), duty_cycle, config.channel);
      continue;
    }

    // Use appropriate conversion based on interface type
    if (config.interface_type == hardware_interface::HW_IF_POSITION)
    {
      duty_cycle = command_to_duty_cycle_position(hw_commands_[i], config);
    }
    else if (config.interface_type == hardware_interface::HW_IF_EFFORT)
    {
      duty_cycle = command_to_duty_cycle_effort(hw_commands_[i]);
    }
    else // velocity interface
    {
      duty_cycle = command_to_duty_cycle_velocity(hw_commands_[i]);
    }

    pca.set_pwm_ms(config.channel, duty_cycle);

    RCLCPP_DEBUG(
      rclcpp::get_logger("Pca9685SystemHardware"),
      "Joint %u (%s): command=%.3f, duty_cycle=%.3f, channel=%d",
      i, config.interface_type.c_str(), hw_commands_[i], duty_cycle, config.channel);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pca9685_hardware_interface::Pca9685SystemHardware, hardware_interface::SystemInterface)
