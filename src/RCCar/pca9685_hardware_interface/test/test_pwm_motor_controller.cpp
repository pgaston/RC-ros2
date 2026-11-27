#include <gtest/gtest.h>

#include <vector>

#include "pca9685_hardware_interface/pwm_motor_controller.hpp"

using pca9685_hardware_interface::PwmMotorController;
using State = PwmMotorController::MotorState;

namespace
{
constexpr double kDt = 0.02;  // 50 Hz control loop
constexpr double kWheelRadius = 0.0508;

// Run the arming sequence to completion with the default config.
PwmMotorController armed(PwmMotorController::Config c = {})
{
  PwmMotorController m;
  m.configure(c);
  for (int i = 0; i < 400 && m.get_state() != State::FORWARD; ++i) {
    m.update(kDt);
  }
  EXPECT_EQ(m.get_state(), State::FORWARD);
  return m;
}

double duty_for(PwmMotorController & m, double command)
{
  m.set_command(command);
  m.update(kDt);
  return m.get_duty_cycle();
}

void step(PwmMotorController & m, double seconds)
{
  for (double t = 0.0; t < seconds; t += kDt) { m.update(kDt); }
}
}  // namespace

TEST(Arming, WalksTheSequenceWithConfiguredDwells)
{
  PwmMotorController m;
  m.configure({});
  m.set_command(5.0);  // a command during arming must be ignored, not remembered as motion

  m.update(kDt);
  EXPECT_EQ(m.get_state(), State::ARMING_NEUTRAL_1);
  EXPECT_DOUBLE_EQ(m.get_duty_cycle(), 1.5);
  EXPECT_DOUBLE_EQ(m.get_velocity(), 0.0);

  step(m, 2.5);
  EXPECT_EQ(m.get_state(), State::ARMING_PULSE);
  m.update(kDt);
  EXPECT_NEAR(m.get_duty_cycle(), 1.5 + 0.05 * 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(m.get_velocity(), 0.0);

  step(m, 0.5);
  EXPECT_EQ(m.get_state(), State::ARMING_NEUTRAL_2);
  EXPECT_DOUBLE_EQ(m.get_duty_cycle(), 1.5);

  step(m, 0.5);
  EXPECT_EQ(m.get_state(), State::FORWARD);
}

TEST(Arming, DwellTimesComeFromConfig)
{
  PwmMotorController::Config c;
  c.arming_neutral_s = 0.1;
  c.arming_pulse_s = 0.1;
  c.arming_settle_s = 0.1;
  PwmMotorController m;
  m.configure(c);
  step(m, 0.35 + 2 * kDt);
  EXPECT_EQ(m.get_state(), State::FORWARD);
}

TEST(Throttle, ForwardTableWithDefaultCalibration)
{
  // defaults: max_wheel_speed 10 rad/s, forward_offset 0.18, max_output 0.40, pulses 1.0/1.5/2.0 ms
  PwmMotorController m = armed();
  EXPECT_DOUBLE_EQ(duty_for(m, 0.0), 1.5);
  EXPECT_DOUBLE_EQ(duty_for(m, 0.005), 1.5);                  // inside the input dead-band
  EXPECT_NEAR(duty_for(m, 0.02), 1.5 + 0.18 * 0.5, 1e-3);     // just moving: lands on the ESC dead-band edge
  EXPECT_NEAR(duty_for(m, 5.0), 1.5 + 0.29 * 0.5, 1e-9);      // half speed: 0.18 + 0.5 * (0.40 - 0.18)
  EXPECT_NEAR(duty_for(m, 10.0), 1.5 + 0.40 * 0.5, 1e-9);     // full speed lands exactly on max_output
  EXPECT_NEAR(duty_for(m, 20.0), 1.5 + 0.40 * 0.5, 1e-9);     // above full: clamped, not wrapped
}

TEST(Throttle, ThreeNavSpeedsAreThreeDifferentPulses)
{
  // Issue #11: /cmd_vel at 0.1, 0.2 and 0.26 m/s must produce visibly different wheel speeds.
  PwmMotorController m = armed();
  std::vector<double> duties;
  for (double v : {0.10, 0.20, 0.26}) {
    duties.push_back(duty_for(m, v / kWheelRadius));
  }
  EXPECT_GT(duties[1] - duties[0], 0.01);
  EXPECT_GT(duties[2] - duties[1], 0.005);
  EXPECT_LT(duties[2], 1.5 + 0.40 * 0.5);  // none of them saturates
}

TEST(Throttle, ReverseMirrorsForwardWithItsOwnOffset)
{
  PwmMotorController m = armed();
  m.set_command(-5.0);
  m.update(kDt);
  EXPECT_EQ(m.get_state(), State::TO_REVERSE_NEUTRAL_1);
  EXPECT_DOUBLE_EQ(m.get_duty_cycle(), 1.5);
  EXPECT_DOUBLE_EQ(m.get_velocity(), 0.0);

  step(m, 0.20 + kDt);
  EXPECT_EQ(m.get_state(), State::TO_REVERSE_PULSE);
  EXPECT_NEAR(m.get_duty_cycle(), 1.5 + 0.10 * 0.5, 1e-9);  // the forward tap the ESC needs, not neutral
  step(m, 0.20 + 0.20 + 2 * kDt);
  EXPECT_EQ(m.get_state(), State::REVERSE);
  EXPECT_NEAR(m.get_duty_cycle(), 1.5 - 0.29 * 0.5, 1e-9);  // -0.18 + (-0.5) * (0.40 - 0.18)
  EXPECT_DOUBLE_EQ(m.get_velocity(), -5.0);

  m.set_command(-10.0);
  m.update(kDt);
  EXPECT_NEAR(m.get_duty_cycle(), 1.5 - 0.40 * 0.5, 1e-9);

  // back to forward: one neutral dwell, then forward throttle
  m.set_command(5.0);
  m.update(kDt);
  EXPECT_EQ(m.get_state(), State::TO_FORWARD_NEUTRAL);
  step(m, 0.20 + 2 * kDt);
  EXPECT_EQ(m.get_state(), State::FORWARD);
  EXPECT_NEAR(m.get_duty_cycle(), 1.5 + 0.29 * 0.5, 1e-9);
}

TEST(Throttle, CalibrationFromConfigNotConstants)
{
  PwmMotorController::Config c;
  c.max_wheel_speed_rad_s = 20.0;
  c.forward_offset = 0.25;
  c.max_output = 0.50;
  c.neutral_pulse_ms = 1.52;
  c.max_pulse_ms = 1.92;
  PwmMotorController m = armed(c);
  EXPECT_NEAR(duty_for(m, 10.0), 1.52 + (0.25 + 0.5 * 0.25) * 0.40, 1e-9);
  EXPECT_NEAR(duty_for(m, 20.0), 1.52 + 0.50 * 0.40, 1e-9);
}

TEST(Watchdog, DisabledByDefaultSoTheMuxOwnsSilence)
{
  PwmMotorController m = armed();
  duty_for(m, 5.0);
  step(m, 5.0);
  EXPECT_NEAR(m.get_duty_cycle(), 1.5 + 0.29 * 0.5, 1e-9);
}

TEST(Watchdog, DecaysToNeutralAfterTimeout)
{
  PwmMotorController::Config c;
  c.watchdog_timeout_s = 0.2;
  PwmMotorController m = armed(c);
  EXPECT_NEAR(duty_for(m, 5.0), 1.5 + 0.29 * 0.5, 1e-9);
  step(m, 0.1);
  EXPECT_NEAR(m.get_duty_cycle(), 1.5 + 0.29 * 0.5, 1e-9);
  step(m, 0.15);
  EXPECT_DOUBLE_EQ(m.get_duty_cycle(), 1.5);
  EXPECT_DOUBLE_EQ(m.get_velocity(), 0.0);
}

TEST(Velocity, IsAnEchoOfTheCommandOnlyWhileDriving)
{
  PwmMotorController m = armed();
  EXPECT_DOUBLE_EQ(m.get_velocity(), 0.0);
  duty_for(m, 3.0);
  EXPECT_DOUBLE_EQ(m.get_velocity(), 3.0);
  duty_for(m, 0.0);
  EXPECT_DOUBLE_EQ(m.get_velocity(), 0.0);
}

TEST(Configure, SanitisesImpossibleValues)
{
  PwmMotorController::Config c;
  c.max_wheel_speed_rad_s = 0.0;
  c.max_output = 1.7;
  c.forward_offset = 0.9;   // above max_output after clamping to 1.0? no: max_output becomes 1.0, offset stays 0.9
  c.reverse_offset = 0.3;   // wrong sign
  PwmMotorController m;
  m.configure(c);
  EXPECT_DOUBLE_EQ(m.config().max_wheel_speed_rad_s, PwmMotorController::Config{}.max_wheel_speed_rad_s);
  EXPECT_DOUBLE_EQ(m.config().max_output, 1.0);
  EXPECT_DOUBLE_EQ(m.config().forward_offset, 0.9);
  EXPECT_DOUBLE_EQ(m.config().reverse_offset, 0.0);
}

namespace
{
PwmMotorController::Config braking_config()
{
  PwmMotorController::Config c;
  c.brake_output = -0.30;
  c.brake_max_s = 0.40;
  c.brake_min_speed_rad_s = 1.0;
  c.brake_release_s = 0.10;
  return c;
}
}  // namespace

TEST(Brake, DisabledByDefaultSoStoppingCoasts)
{
  PwmMotorController m = armed();
  duty_for(m, 10.0);
  EXPECT_DOUBLE_EQ(duty_for(m, 0.0), 1.5);
  EXPECT_EQ(m.get_state(), State::FORWARD);
}

TEST(Brake, StopFromFullSpeedBrakesForBrakeMaxThenReleases)
{
  PwmMotorController m = armed(braking_config());
  duty_for(m, 10.0);
  EXPECT_NEAR(duty_for(m, 0.0), 1.5 - 0.30 * 0.5, 1e-9);  // below neutral: brake, not reverse
  EXPECT_EQ(m.get_state(), State::BRAKING);
  EXPECT_DOUBLE_EQ(m.get_velocity(), 0.0);

  step(m, 0.40 - 2 * kDt);
  EXPECT_EQ(m.get_state(), State::BRAKING);
  step(m, 2 * kDt);
  EXPECT_EQ(m.get_state(), State::BRAKE_RELEASE);
  EXPECT_DOUBLE_EQ(m.get_duty_cycle(), 1.5);

  step(m, 0.10 + kDt);
  EXPECT_EQ(m.get_state(), State::FORWARD);
  EXPECT_DOUBLE_EQ(m.get_duty_cycle(), 1.5);

  // Already stopped: holding zero must not brake again (that would be reverse on a stopped motor).
  step(m, 0.5);
  EXPECT_EQ(m.get_state(), State::FORWARD);
  EXPECT_DOUBLE_EQ(m.get_duty_cycle(), 1.5);
}

TEST(Brake, BrakeLengthScalesWithLastSpeed)
{
  PwmMotorController m = armed(braking_config());
  duty_for(m, 5.0);   // half of max_wheel_speed -> 0.20 s of brake
  duty_for(m, 0.0);
  EXPECT_EQ(m.get_state(), State::BRAKING);
  step(m, 0.20 + kDt);
  EXPECT_EQ(m.get_state(), State::BRAKE_RELEASE);
}

TEST(Brake, SlowCommandsJustCoast)
{
  PwmMotorController m = armed(braking_config());
  duty_for(m, 0.5);   // below brake_min_speed_rad_s
  EXPECT_DOUBLE_EQ(duty_for(m, 0.0), 1.5);
  EXPECT_EQ(m.get_state(), State::FORWARD);
}

TEST(Brake, ForwardCommandDuringBrakeResumesImmediately)
{
  PwmMotorController m = armed(braking_config());
  duty_for(m, 10.0);
  duty_for(m, 0.0);
  EXPECT_EQ(m.get_state(), State::BRAKING);
  EXPECT_NEAR(duty_for(m, 5.0), 1.5 + 0.29 * 0.5, 1e-9);
  EXPECT_EQ(m.get_state(), State::FORWARD);
  EXPECT_DOUBLE_EQ(m.get_velocity(), 5.0);
}

TEST(Brake, ReverseFromForwardBrakesThenUsesTheTapSequence)
{
  PwmMotorController m = armed(braking_config());
  duty_for(m, 10.0);
  EXPECT_NEAR(duty_for(m, -5.0), 1.5 - 0.30 * 0.5, 1e-9);
  EXPECT_EQ(m.get_state(), State::BRAKING);
  step(m, 0.40 + kDt);
  EXPECT_EQ(m.get_state(), State::BRAKE_RELEASE);
  step(m, 0.10 + kDt);
  EXPECT_EQ(m.get_state(), State::TO_REVERSE_NEUTRAL_1);
  step(m, 0.20 + kDt);
  EXPECT_EQ(m.get_state(), State::TO_REVERSE_PULSE);
  step(m, 0.40 + 2 * kDt);
  EXPECT_EQ(m.get_state(), State::REVERSE);
  m.update(kDt);
  EXPECT_NEAR(m.get_duty_cycle(), 1.5 - 0.29 * 0.5, 1e-9);
}

TEST(Brake, ConfigureSanitisesBrakeOutputSign)
{
  PwmMotorController::Config c = braking_config();
  c.brake_output = 0.3;  // positive would be throttle, not brake
  PwmMotorController m;
  m.configure(c);
  EXPECT_DOUBLE_EQ(m.config().brake_output, 0.0);
}

// The ESC ignores forward after power-up until it has seen one below-neutral
// pulse (#26). Arming ends with that pulse when arming_reverse_output < 0.
namespace
{
PwmMotorController::Config unlock_config()
{
  PwmMotorController::Config c;
  c.arming_reverse_output = -0.10;
  c.arming_reverse_s = 0.20;
  c.arming_reverse_settle_s = 0.30;
  return c;
}
}  // namespace

TEST(ArmingUnlock, DisabledByDefaultSoArmingGoesStraightToForward)
{
  PwmMotorController m;
  m.configure({});
  step(m, 2.5 + 0.5 + 0.5 + 2 * kDt);
  EXPECT_EQ(m.get_state(), State::FORWARD);
}

TEST(ArmingUnlock, ForwardTapThenBelowNeutralPulseThenNeutralBeforeForward)
{
  PwmMotorController m;
  m.configure(unlock_config());
  m.set_command(5.0);  // still ignored until FORWARD

  step(m, 2.5 + 0.5 + 0.5 + 2 * kDt);
  // The tap comes first: without it the fresh ESC drives the pulse as reverse.
  EXPECT_EQ(m.get_state(), State::ARMING_TAP);
  m.update(kDt);
  EXPECT_NEAR(m.get_duty_cycle(), 1.5 + 0.10 * 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(m.get_velocity(), 0.0);

  step(m, 0.20);
  EXPECT_EQ(m.get_state(), State::ARMING_TAP_NEUTRAL);
  EXPECT_DOUBLE_EQ(m.get_duty_cycle(), 1.5);

  step(m, 0.20);
  EXPECT_EQ(m.get_state(), State::ARMING_REVERSE_PULSE);
  m.update(kDt);
  EXPECT_NEAR(m.get_duty_cycle(), 1.5 - 0.10 * 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(m.get_velocity(), 0.0);

  step(m, 0.20);
  EXPECT_EQ(m.get_state(), State::ARMING_NEUTRAL_3);
  EXPECT_DOUBLE_EQ(m.get_duty_cycle(), 1.5);
  EXPECT_DOUBLE_EQ(m.get_velocity(), 0.0);

  step(m, 0.30);
  EXPECT_EQ(m.get_state(), State::FORWARD);
  m.update(kDt);
  EXPECT_GT(m.get_duty_cycle(), 1.5);  // forward accepted straight after
}

TEST(ArmingUnlock, ConfigureSanitisesPulseSign)
{
  PwmMotorController::Config c = unlock_config();
  c.arming_reverse_output = 0.10;  // above neutral would be throttle, not the unlock
  PwmMotorController m;
  m.configure(c);
  EXPECT_DOUBLE_EQ(m.config().arming_reverse_output, 0.0);
  step(m, 2.5 + 0.5 + 0.5 + 2 * kDt);
  EXPECT_EQ(m.get_state(), State::FORWARD);
}
