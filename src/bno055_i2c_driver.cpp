/* bno055_i2c_driver.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 */

#include "imu_bno055/bno055_i2c_driver.h"

namespace imu_bno055
{

BNO055I2CDriver::BNO055I2CDriver(std::string device_, int address_) : device(device_), address(address_)
{
}

bool BNO055I2CDriver::reset()
{
  int i = 0;

  ROS_DEBUG("Setting operation mode to CONFIG");
  int result = _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
  if (result < 0)
  {
    ROS_ERROR("Failed to set operation mode to CONFIG");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  ROS_DEBUG("Sending reset command to IMU");
  result = _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0x20);
  if (result < 0)
  {
    ROS_ERROR("Failed to send reset command to IMU");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  ROS_DEBUG("Waiting for IMU to come back online");
  while (_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (i++ > 500)
    {
      ROS_ERROR("Chip did not come back online within 5 seconds of reset");
      return false;
    }
  }
  ROS_DEBUG("IMU is back online");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  ROS_DEBUG("Setting normal power mode");
  result = _i2c_smbus_write_byte_data(file, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
  if (result < 0)
  {
    ROS_ERROR("Failed to set normal power mode");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  ROS_DEBUG("Setting page ID to 0");
  result = _i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 0);
  if (result < 0)
  {
    ROS_ERROR("Failed to set page ID to 0");
    return false;
  }
  result = _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0);
  if (result < 0)
  {
    ROS_ERROR("Failed to clear SYS_TRIGGER");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  ROS_DEBUG("Setting operation mode to NDOF");
  result = _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
  if (result < 0)
  {
    ROS_ERROR("Failed to set operation mode to NDOF");
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  ROS_INFO("IMU reset completed successfully");
  return true;
}

void BNO055I2CDriver::init()
{
  file = open(device.c_str(), O_RDWR);
  if (file < 0)
    throw std::runtime_error("i2c device open failed");

  if (ioctl(file, I2C_SLAVE, address) < 0)
    throw std::runtime_error("i2c device open failed");

  if (_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID)
    throw std::runtime_error("incorrect chip ID");

  ROS_DEBUG(
      "rev ids: accel: %d mag: %d gyro: %d sw: %d bl: %d", _i2c_smbus_read_byte_data(file, BNO055_ACCEL_REV_ID_ADDR),
      _i2c_smbus_read_byte_data(file, BNO055_MAG_REV_ID_ADDR), _i2c_smbus_read_byte_data(file, BNO055_GYRO_REV_ID_ADDR),
      _i2c_smbus_read_word_data(file, BNO055_SW_REV_ID_LSB_ADDR),
      _i2c_smbus_read_byte_data(file, BNO055_BL_REV_ID_ADDR));

  if (!reset())
    throw std::runtime_error("chip init failed");
}

IMURecord BNO055I2CDriver::read()
{
  IMURecord record;

  // can only read a length of 0x20 at a time, so do it in 2 reads
  // BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block that aligns with the IMURecord struct
  if (_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20)
    throw std::runtime_error("read error");
  if (_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13, (uint8_t*)&record + 0x20) != 0x13)
    throw std::runtime_error("read error");

  return record;
}

}  // namespace imu_bno055
