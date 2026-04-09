#ifndef ICM20948_HARDWARE__ICM20948_INTERFACE_HPP_
#define ICM20948_HARDWARE__ICM20948_INTERFACE_HPP_

#include <cstdint>
#include <string>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

#include "icm20948/ICM_20948_C.h"

#define GRAVITY 9.80665
#define DEG_TO_RAD 0.017453292519943295

namespace icm20948_hardware {
class ICM20948Interface : public hardware_interface::SensorInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ICM20948Interface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

private:
  /**
   * @brief File descriptor for the I2C device. Not initialized if -1.
   */
  int i2c_fd_ = -1;

  /**
   * @brief The I2C device file, defaults to /dev/i2c-1.
   */
  std::string i2c_device_ = "/dev/i2c-1";

  /**
   * @brief The I2C address of the device, defaults to 0x69.
   */
  int i2c_address_ = 0x69;

  /**
   * @brief The name of the sensor, used for logging and identifying the sensor.
   */
  std::string sensor_name_;

  bool use_dlpf_ = true;

  uint8_t accel_range_ = 0;
  uint8_t gyro_range_ = 0;

  std::array<double, 3> accel_scales_;
  std::array<double, 3> gyro_scales_;

  std::array<double, 9> hw_states_;

  // ICM20948 structs

  /**
   * @brief The ICM20948 device struct, used for reading and configuring IMU.
   */
  ICM_20948_Device_t icm_device_;

  /**
   * @brief The ICM20948 serial interface struct, used for providing I2C read/write callbacks.
   */
  ICM_20948_Serif_t icm_serif_;

  // static ICM20948 serial interface callbacks

  /**
   * @brief Static I2C write callback function, used by the ICM20948 serial interface struct.
   */
  static ICM_20948_Status_e i2c_write_cb(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);

  /**
   * @brief Static I2C read callback function, used by the ICM20948 serial interface struct.
   */
  static ICM_20948_Status_e i2c_read_cb(uint8_t regaddr, uint8_t *pdata, uint32_t len, void *user);
};
} // namespace icm20948_hardware

#endif // ICM20948_HARDWARE__ICM20948_INTERFACE_HPP_
