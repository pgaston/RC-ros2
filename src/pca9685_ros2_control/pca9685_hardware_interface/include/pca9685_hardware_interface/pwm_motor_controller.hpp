#ifndef PCA9685_HARDWARE_INTERFACE__PWM_MOTOR_CONTROLLER_HPP_
#define PCA9685_HARDWARE_INTERFACE__PWM_MOTOR_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>

namespace pca9685_hardware_interface
{

class PwmMotorController
{
public:
  enum class MotorState {
    INITIALIZING,
    ARMING_NEUTRAL_1,
    ARMING_PULSE,
    ARMING_NEUTRAL_2,
    FORWARD,
    REVERSE,
    TO_REVERSE_NEUTRAL_1,
    TO_REVERSE_PULSE,
    TO_REVERSE_NEUTRAL_2,
    TO_FORWARD_NEUTRAL
  };

  struct Config {
    // Input deadband (ignore small commands from stick/nav)
    double input_deadband = 0.01; 
    
    // Output scaling/offset parameters
    double max_speed_scale = 0.4;        // Scale factor for input
    double forward_offset = 0.271;       // Added to output if forward
    double reverse_offset = -0.0405;     // Added to output if reverse

    double min_pwm_duty = 1.0;     // 1.0 ms
    double neutral_pwm_duty = 1.5; // 1.5 ms
    double max_pwm_duty = 2.0;     // 2.0 ms
    double watchdog_timeout = 0.2; // seconds
  };

  PwmMotorController();
  
  void configure(const Config& config);
  void set_command(double command);
  void update();
  double get_duty_cycle() const;
  MotorState get_state() const { return state_; }

private:
  Config config_;
  
  MotorState state_ = MotorState::INITIALIZING;
  std::chrono::steady_clock::time_point last_command_time_;
  std::chrono::steady_clock::time_point state_entry_time_;
  
  double target_command_ = 0.0;
  double current_duty_cycle_ = 1.5;
  
  // Helper to check if command is basically zero
  bool is_command_neutral() const;
  bool is_command_forward() const;
  bool is_command_reverse() const;

  // Process specific sequences
  void process_state_machine(double dt);
  double compute_duty_cycle(double command);

  double get_velocity() const;
};

} // namespace pca9685_hardware_interface

#endif // PCA9685_HARDWARE_INTERFACE__PWM_MOTOR_CONTROLLER_HPP_
