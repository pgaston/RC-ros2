#include "pca9685_hardware_interface/pwm_motor_controller.hpp"

namespace pca9685_hardware_interface
{

PwmMotorController::PwmMotorController()
{
  last_command_time_ = std::chrono::steady_clock::now();
  state_entry_time_ = std::chrono::steady_clock::now();
}

void PwmMotorController::configure(const Config& config)
{
  config_ = config;
  // Reset state to initializing
  state_ = MotorState::INITIALIZING;
  state_entry_time_ = std::chrono::steady_clock::now();
  current_duty_cycle_ = config_.neutral_pwm_duty;
}

void PwmMotorController::set_command(double command)
{
  target_command_ = command;
  last_command_time_ = std::chrono::steady_clock::now();
}

bool PwmMotorController::is_command_neutral() const
{
  return std::abs(target_command_) < config_.input_deadband;
}

bool PwmMotorController::is_command_forward() const
{
  return target_command_ >= config_.input_deadband;
}

bool PwmMotorController::is_command_reverse() const
{
  return target_command_ <= -config_.input_deadband;
}

double PwmMotorController::compute_duty_cycle(double command)
{
  // 1. Input Deadband filtering
  if (std::abs(command) < config_.input_deadband) {
    return config_.neutral_pwm_duty;
  }

  // 2. Calculate Final Speed (Output) with Asymmetric Deadbands
  // Formula: output = deadband_offset + (input * max_speed_scale)
  double final_speed = 0.0;
  
  if (command > 0) {
    final_speed = config_.forward_offset + (command * config_.max_speed_scale);
  } else {
    // command is negative, so this subtracts from offset (or adds if offset is negative)
    // Wait, let's match the python logic:
    // final_speed = self.deadbandReverse + (input_speed * self.max_speed)
    // deadbandReverse is -0.0405
    final_speed = config_.reverse_offset + (command * config_.max_speed_scale);
  }

  // 3. Clamp final speed to [-max_output, max_output]
  double limit = std::abs(config_.max_output);
  final_speed = std::max(-limit, std::min(limit, final_speed));

  // 4. Map [-1, 1] to PWM Duty Cycle (min_pwm_duty to max_pwm_duty)
  // neutral_pwm_duty is center (0.0)
  
  if (std::abs(final_speed) < 0.01) {
      // Should not happen given offsets, but safety check
      return config_.neutral_pwm_duty;
  } else if (final_speed > 0) {
    // Forward direction: interpolate [0, 1] -> [neutral, max]
    // Ratio is final_speed directly since it's 0..1
    return config_.neutral_pwm_duty + final_speed * (config_.max_pwm_duty - config_.neutral_pwm_duty);
  } else {
    // Reverse direction: interpolate [0, -1] -> [neutral, min]
    // Ratio is -final_speed (positive 0..1)
    return config_.neutral_pwm_duty + final_speed * (config_.neutral_pwm_duty - config_.min_pwm_duty);
  }
}

void PwmMotorController::update()
{
  auto now = std::chrono::steady_clock::now();
  
  // 1. Watchdog check (only if timeout is positive)
  if (config_.watchdog_timeout > 0.0) {
    double time_since_command = std::chrono::duration<double>(now - last_command_time_).count();
    if (time_since_command > config_.watchdog_timeout) {
      target_command_ = 0.0;
    }
  }
  
  double dt_state = std::chrono::duration<double>(now - state_entry_time_).count();
  
  // 2. State Machine
  switch (state_)
  {
    case MotorState::INITIALIZING:
      // Start Arming sequence immediately
      state_ = MotorState::ARMING_NEUTRAL_1;
      state_entry_time_ = now;
      current_duty_cycle_ = config_.neutral_pwm_duty;
      break;

    case MotorState::ARMING_NEUTRAL_1:
      current_duty_cycle_ = config_.neutral_pwm_duty;
      if (dt_state >= 0.5) {
        state_ = MotorState::ARMING_PULSE;
        state_entry_time_ = now;
      }
      break;

    case MotorState::ARMING_PULSE:
      // Output raw 0.1 positive (sub-deadband pulse for ESC detection only)
      // Do NOT use compute_duty_cycle as it adds the motion offset
      current_duty_cycle_ = config_.neutral_pwm_duty + 0.1 * (config_.max_pwm_duty - config_.neutral_pwm_duty);
      if (dt_state >= 0.5) {
        state_ = MotorState::ARMING_NEUTRAL_2;
        state_entry_time_ = now;
      }
      break;

    case MotorState::ARMING_NEUTRAL_2:
      current_duty_cycle_ = config_.neutral_pwm_duty;
      if (dt_state >= 0.5) {
        // Arming done. Default to FORWARD state (idle)
        state_ = MotorState::FORWARD;
        state_entry_time_ = now;
      }
      break;

    case MotorState::FORWARD:
      if (is_command_reverse()) {
        // Transition to Reverse needed
        state_ = MotorState::TO_REVERSE_NEUTRAL_1;
        state_entry_time_ = now;
        current_duty_cycle_ = config_.neutral_pwm_duty;
      } else {
        // Normal operation (Positive or Neutral)
        current_duty_cycle_ = compute_duty_cycle(target_command_);
      }
      break;

    case MotorState::TO_REVERSE_NEUTRAL_1:
      current_duty_cycle_ = config_.neutral_pwm_duty;
      if (dt_state >= 0.5) {
        state_ = MotorState::TO_REVERSE_PULSE;
        state_entry_time_ = now;
      }
      break;

    case MotorState::TO_REVERSE_PULSE:
      // Output raw 0.1 positive (sub-deadband pulse for ESC detection only)
      current_duty_cycle_ = config_.neutral_pwm_duty + 0.1 * (config_.max_pwm_duty - config_.neutral_pwm_duty);
      if (dt_state >= 0.5) {
        state_ = MotorState::TO_REVERSE_NEUTRAL_2;
        state_entry_time_ = now;
      }
      break;
      
    case MotorState::TO_REVERSE_NEUTRAL_2:
      current_duty_cycle_ = config_.neutral_pwm_duty;
      if (dt_state >= 0.5) {
        state_ = MotorState::REVERSE;
        state_entry_time_ = now;
      }
      break;

    case MotorState::REVERSE:
      if (is_command_forward()) {
        // Transition to Forward needed
        state_ = MotorState::TO_FORWARD_NEUTRAL;
        state_entry_time_ = now;
        current_duty_cycle_ = config_.neutral_pwm_duty;
      } else {
        // Normal operation (Reverse or Neutral)
        current_duty_cycle_ = compute_duty_cycle(target_command_);
      }
      break;

    case MotorState::TO_FORWARD_NEUTRAL:
      current_duty_cycle_ = config_.neutral_pwm_duty;
      if (dt_state >= 0.5) {
        state_ = MotorState::FORWARD;
        state_entry_time_ = now;
      }
      break;
  }
}

double PwmMotorController::get_duty_cycle() const
{
  return current_duty_cycle_;
}

double PwmMotorController::get_velocity() const
{
  // If output is essentially neutral (stopped), report 0 velocity
  // This handles deadband, watchdog, and neutral states
  if (std::abs(current_duty_cycle_ - config_.neutral_pwm_duty) < 0.001) {
    return 0.0;
  }
  
  // If we are in initialization/arming/transition sequences (pulsing),
  // we do not report this as effective velocity for odometry.
  if (state_ != MotorState::FORWARD && state_ != MotorState::REVERSE) {
    return 0.0;
  }

  // Otherwise, return the command that resulted in this motion
  // (We return the requested command, not the scaled/offset internal value,
  // as the controller loop expects units matching the command)
  return target_command_;
}

} // namespace pca9685_hardware_interface
