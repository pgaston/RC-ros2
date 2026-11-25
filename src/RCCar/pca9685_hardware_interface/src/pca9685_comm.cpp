#include "pca9685_hardware_interface/pca9685_comm.h"
#include <unistd.h>
#include <cmath>

#include "pca9685_hardware_interface/Constants.h"
#include "pca9685_hardware_interface/I2CPeripheral.h"

namespace PiPCA9685 {

PCA9685::PCA9685(const std::string &device, int address) {
  i2c_dev = std::make_unique<I2CPeripheral>(device, address);

  // Auto-increment first, so every channel update below is one I2C
  // transaction. Outputs change on each I2C STOP (MODE2 OCH = 0); written a
  // byte at a time, a channel shows a half-updated pulse width between bytes
  // (e.g. 1.25 ms, half reverse, while a 1.5 ms neutral is being cleared).
  // RESTART is masked so this write cannot restart the PWM channels.
  const auto mode1_initial = i2c_dev->ReadRegisterByte(MODE1);
  i2c_dev->WriteRegisterByte(MODE1, (mode1_initial & ~RESTART) | AI);

  set_all_pwm(0,0);
  i2c_dev->WriteRegisterByte(MODE2, OUTDRV);
  i2c_dev->WriteRegisterByte(MODE1, ALLCALL | AI);
  usleep(5'000);
  auto mode1_val = i2c_dev->ReadRegisterByte(MODE1);
  mode1_val &= ~SLEEP;
  i2c_dev->WriteRegisterByte(MODE1, mode1_val);
  usleep(5'000);
}

PCA9685::~PCA9685() = default;

void PCA9685::set_pwm_freq(const double freq_hz) {
  frequency = freq_hz;

  auto prescaleval = 2.5e7; //    # 25MHz
  prescaleval /= 4096.0; //       # 12-bit
  prescaleval /= freq_hz;
  prescaleval -= 1.0;

  auto prescale = static_cast<int>(std::round(prescaleval));

  const auto oldmode = i2c_dev->ReadRegisterByte(MODE1);

  auto newmode = (oldmode & 0x7F) | SLEEP;

  i2c_dev->WriteRegisterByte(MODE1, newmode);
  i2c_dev->WriteRegisterByte(PRESCALE, prescale);
  i2c_dev->WriteRegisterByte(MODE1, oldmode);
  usleep(5'000);
  i2c_dev->WriteRegisterByte(MODE1, oldmode | RESTART);
}

void PCA9685::set_pwm(const int channel, const uint16_t on, const uint16_t off) {
  const uint8_t bytes[4] = {
    static_cast<uint8_t>(on & 0xFF), static_cast<uint8_t>(on >> 8),
    static_cast<uint8_t>(off & 0xFF), static_cast<uint8_t>(off >> 8)};
  i2c_dev->WriteRegisterBlock(LED0_ON_L + 4 * channel, bytes, 4);
}

void PCA9685::set_all_pwm(const uint16_t on, const uint16_t off) {
  const uint8_t bytes[4] = {
    static_cast<uint8_t>(on & 0xFF), static_cast<uint8_t>(on >> 8),
    static_cast<uint8_t>(off & 0xFF), static_cast<uint8_t>(off >> 8)};
  i2c_dev->WriteRegisterBlock(ALL_LED_ON_L, bytes, 4);
}

void PCA9685::set_pwm_ms(const int channel, const double ms) {
  auto period_ms = 1000.0 / frequency;
  auto bits_per_ms = 4096 / period_ms;
  auto bits = ms * bits_per_ms;
  set_pwm(channel, 0, bits);
}

}  // namespace PiPCA9685
