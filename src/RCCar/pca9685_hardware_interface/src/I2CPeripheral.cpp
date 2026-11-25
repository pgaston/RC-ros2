#include "pca9685_hardware_interface/I2CPeripheral.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
extern "C" {
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}
#include <stdexcept>
#include <system_error>

namespace PiPCA9685 {

I2CPeripheral::I2CPeripheral(const std::string& device, const uint8_t address) {
  OpenBus(device);
  ConnectToPeripheral(address);
}

I2CPeripheral::~I2CPeripheral() {
  close(bus_fd);
}

void I2CPeripheral::WriteRegisterByte(const uint8_t register_address, const uint8_t value) {
  i2c_smbus_data data;
  data.byte = value;
  const auto err = i2c_smbus_access(bus_fd, I2C_SMBUS_WRITE, register_address, I2C_SMBUS_BYTE_DATA, &data);
  if(err) {
    const auto msg = "Could not write value (" + std::to_string(value) + ") to register " + std::to_string(register_address);
    throw std::system_error(errno, std::system_category(), msg);
  }
}

void I2CPeripheral::WriteRegisterBlock(const uint8_t register_address, const uint8_t* values, const uint8_t len) {
  if (len > I2C_SMBUS_BLOCK_MAX) {
    throw std::invalid_argument("I2C block write longer than " + std::to_string(I2C_SMBUS_BLOCK_MAX) + " bytes");
  }
  i2c_smbus_data data;
  data.block[0] = len;
  for (uint8_t i = 0; i < len; ++i) {
    data.block[i + 1] = values[i];
  }
  const auto err = i2c_smbus_access(bus_fd, I2C_SMBUS_WRITE, register_address, I2C_SMBUS_I2C_BLOCK_DATA, &data);
  if(err) {
    const auto msg = "Could not write " + std::to_string(len) + " bytes from register " + std::to_string(register_address);
    throw std::system_error(-err, std::system_category(), msg);
  }
}

uint8_t I2CPeripheral::ReadRegisterByte(const uint8_t register_address) {
  i2c_smbus_data data;
  const auto err = i2c_smbus_access(bus_fd, I2C_SMBUS_READ, register_address, I2C_SMBUS_BYTE_DATA, &data);
  if(err) {
    const auto msg = "Could not read value at register " + std::to_string(register_address);
    throw std::system_error(-err, std::system_category(), msg);
  }
  return data.byte & 0xFF;
}

void I2CPeripheral::OpenBus(const std::string& device) {
  bus_fd = open(device.c_str(), O_RDWR);
  if(bus_fd < 0) {
    throw std::system_error(errno, std::system_category(), "Could not open i2c bus.");
  }
}

void I2CPeripheral::ConnectToPeripheral(const uint8_t address) {
  if(ioctl(bus_fd, I2C_SLAVE, address) < 0) {
    throw std::system_error(errno, std::system_category(), "Could not set peripheral address.");
  }
}

}  // namespace PiPCA9685
