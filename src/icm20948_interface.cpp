#include <array>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "icm20948/ICM_20948_C.h"

#include "icm20948_hardware/icm20948_interface.hpp"

namespace {

bool parse_bool(const std::string & s) {
  std::string lower_s = s;
  std::transform(lower_s.begin(), lower_s.end(), lower_s.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (lower_s == "true" || lower_s == "1") {
    return true;
  } else if (lower_s == "false" || lower_s == "0") {
    return false;
  }
  throw std::invalid_argument("Failed to parse boolean from: " + s);
}

std::vector<bool> parse_bool_array(const std::string & s) {
  std::vector<bool> result;
  std::string inner = s;

  // Remove brackets if they exist
  inner.erase(std::remove(inner.begin(), inner.end(), '['), inner.end());
  inner.erase(std::remove(inner.begin(), inner.end(), ']'), inner.end());

  std::stringstream ss(inner);
  std::string item;
  while (std::getline(ss, item, ',')) {
    // Trim leading/trailing whitespace
    item.erase(0, item.find_first_not_of(" \t\n\r"));
    item.erase(item.find_last_not_of(" \t\n\r") + 1);
    if (!item.empty()) {
      result.push_back(parse_bool(item));
    }
  }
  return result;
}

} // namespace


namespace icm20948_hardware {

std::vector<hardware_interface::StateInterface>
ICM20948Interface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(9);

  state_interfaces.emplace_back(sensor_name_, "linear_acceleration.x", &hw_states_[0]);
  state_interfaces.emplace_back(sensor_name_, "linear_acceleration.y", &hw_states_[1]);
  state_interfaces.emplace_back(sensor_name_, "linear_acceleration.z", &hw_states_[2]);
  state_interfaces.emplace_back(sensor_name_, "angular_velocity.x", &hw_states_[3]);
  state_interfaces.emplace_back(sensor_name_, "angular_velocity.y", &hw_states_[4]);
  state_interfaces.emplace_back(sensor_name_, "angular_velocity.z", &hw_states_[5]);
  state_interfaces.emplace_back(sensor_name_, "magnetic_field.x", &hw_states_[6]);
  state_interfaces.emplace_back(sensor_name_, "magnetic_field.y", &hw_states_[7]);
  state_interfaces.emplace_back(sensor_name_, "magnetic_field.z", &hw_states_[8]);
  state_interfaces.emplace_back(sensor_name_, "orientation.x", &hw_states_[9]);
  state_interfaces.emplace_back(sensor_name_, "orientation.y", &hw_states_[10]);
  state_interfaces.emplace_back(sensor_name_, "orientation.z", &hw_states_[11]);
  state_interfaces.emplace_back(sensor_name_, "orientation.w", &hw_states_[12]);

  return state_interfaces;
}

ICM_20948_Status_e ICM20948Interface::i2c_write_cb(uint8_t regaddr,
                                                   uint8_t *pdata, uint32_t len,
                                                   void *user) {
  // Get pointer to ICM20948 class
  auto *hw = static_cast<ICM20948Interface *>(user);

  // Exit if I2C file not open
  if (hw->i2c_fd_ < 0) {
    return ICM_20948_Stat_Err;
  }

  // Initialize buffer
  std::vector<uint8_t> buf;
  buf.reserve(len + 1);

  // Write register address
  buf.push_back(regaddr);
  // Write from pdata
  buf.insert(buf.end(), pdata, pdata + len);

  // Perform write
  if (::write(hw->i2c_fd_, buf.data(), buf.size()) !=
      static_cast<ssize_t>(buf.size())) {
    return ICM_20948_Stat_Err;
  }

  return ICM_20948_Stat_Ok;
}

ICM_20948_Status_e ICM20948Interface::i2c_read_cb(uint8_t regaddr,
                                                  uint8_t *pdata, uint32_t len,
                                                  void *user) {
  // Get pointer to ICM20948 class
  auto *hw = static_cast<ICM20948Interface *>(user);

  // Exit if I2C file not open
  if (hw->i2c_fd_ < 0) {
    return ICM_20948_Stat_Err;
  }

  // Write register address
  if (::write(hw->i2c_fd_, &regaddr, 1) != 1) {
    return ICM_20948_Stat_Err;
  }

  // Read into pdata
  if (::read(hw->i2c_fd_, pdata, len) != static_cast<ssize_t>(len)) {
    return ICM_20948_Stat_Err;
  }

  return ICM_20948_Stat_Ok;
}

hardware_interface::CallbackReturn ICM20948Interface::on_init(
    const hardware_interface::HardwareInfo &hardware_info) {
  if (hardware_interface::SensorInterface::on_init(hardware_info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.sensors.size() != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("ICM20948Interface"),
                 "Expected 1 sensor, got %zu", info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  i2c_device_ = info_.hardware_parameters["i2c_device"];
  sensor_name_ = info_.sensors[0].name;

  std::vector<bool> reverse_accel = {false, false, false};
  std::vector<bool> reverse_gyro = {false, false, false};

  if (info_.hardware_parameters.find("use_dlpf") !=
      info_.hardware_parameters.end()) {
    use_dlpf_ =
        hardware_interface::parse_bool(info_.hardware_parameters["use_dlpf"]);
  }

  if (info_.hardware_parameters.find("i2c_address") !=
      info_.hardware_parameters.end()) {
    i2c_address_ = std::stoi(
        info_.hardware_parameters["i2c_address"]);
  }

  if (info_.hardware_parameters.find("reverse_accel") !=
      info_.hardware_parameters.end()) {
    reverse_accel = parse_bool_array(
        info_.hardware_parameters["reverse_accel"]);
  }

  if (info_.hardware_parameters.find("reverse_gyro") !=
      info_.hardware_parameters.end()) {
    reverse_gyro = parse_bool_array(
        info_.hardware_parameters["reverse_gyro"]);
  }

  if (info_.hardware_parameters.find("accel_range") !=
      info_.hardware_parameters.end()) {
    accel_range_ = std::stoi(
        info_.hardware_parameters["accel_range"]);
  }

  if (info_.hardware_parameters.find("gyro_range") !=
      info_.hardware_parameters.end()) {
    gyro_range_ = std::stoi(
        info_.hardware_parameters["gyro_range"]);
  }

  double accel_scale = (GRAVITY * pow(2.0, accel_range_)) / 16384.0;
  double gyro_scale = (DEG_TO_RAD * pow(2.0, gyro_range_)) / 131.0;

  accel_scales_[0] = reverse_accel[0] ? -accel_scale : accel_scale;
  accel_scales_[1] = reverse_accel[1] ? -accel_scale : accel_scale;
  accel_scales_[2] = reverse_accel[2] ? -accel_scale : accel_scale;
  gyro_scales_[0] = reverse_gyro[0] ? -gyro_scale : gyro_scale;
  gyro_scales_[1] = reverse_gyro[1] ? -gyro_scale : gyro_scale;
  gyro_scales_[2] = reverse_gyro[2] ? -gyro_scale : gyro_scale;

  // Set up the vtable
  icm_serif_.write = i2c_write_cb;
  icm_serif_.read = i2c_read_cb;
  icm_serif_.user = this;

  hw_states_.fill(std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ICM20948Interface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  i2c_fd_ = open(i2c_device_.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ICM20948Interface"),
                 "Failed to open I2C device");
    close(i2c_fd_);
    i2c_fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("ICM20948Interface"),
                 "Failed to set I2C address");
    close(i2c_fd_);
    i2c_fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  ICM_20948_init_struct(&icm_device_);
  ICM_20948_link_serif(&icm_device_, &icm_serif_);

  while (ICM_20948_check_id(&icm_device_) != ICM_20948_Stat_Ok) {
    RCLCPP_WARN(rclcpp::get_logger("ICM20948Interface"),
                "Failed to verify ICM20948 ID, retrying...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  ICM_20948_sw_reset(&icm_device_);
  std::this_thread::sleep_for(std::chrono::milliseconds(250));

  // Set Gyro and Accelerometer to a particular sample mode
  ICM_20948_set_sample_mode(
      &icm_device_,
      (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc |
                                      ICM_20948_Internal_Gyr),
      ICM_20948_Sample_Mode_Continuous); // optional:
                                         // ICM_20948_Sample_Mode_Continuous.
                                         // ICM_20948_Sample_Mode_Cycled

  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myfss;
  myfss.a = accel_range_; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  myfss.g = gyro_range_;  // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  ICM_20948_set_full_scale(
      &icm_device_,
      (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc |
                                      ICM_20948_Internal_Gyr),
      myfss);

  // Set up DLPF configuration
  ICM_20948_dlpcfg_t myDLPcfg;
  myDLPcfg.a = acc_d473bw_n499bw;
  myDLPcfg.g = gyr_d361bw4_n376bw5;
  ICM_20948_set_dlpf_cfg(
      &icm_device_,
      (ICM_20948_InternalSensorID_bm)(ICM_20948_Internal_Acc |
                                      ICM_20948_Internal_Gyr),
      myDLPcfg);

  // Choose whether or not to use DLPF
  ICM_20948_enable_dlpf(&icm_device_, ICM_20948_Internal_Acc, use_dlpf_);
  ICM_20948_enable_dlpf(&icm_device_, ICM_20948_Internal_Gyr, use_dlpf_);

  // Now wake the sensor up
  ICM_20948_sleep(&icm_device_, false);
  ICM_20948_low_power(&icm_device_, false);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ICM20948Interface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
    i2c_fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
ICM20948Interface::read(const rclcpp::Time & /*time*/,
                        const rclcpp::Duration & /*period*/) {
  ICM_20948_AGMT_t agmt;

  // Read sensor data (assuming some data update function, simple raw read for
  // example)
  if (ICM_20948_get_agmt(&icm_device_, &agmt) != ICM_20948_Stat_Ok) {
    RCLCPP_ERROR(rclcpp::get_logger("ICM20948Interface"),
                 "Failed to read ICM20948 sensor data");
    return hardware_interface::return_type::ERROR;
  }

  hw_states_[0] = static_cast<double>(agmt.acc.axes.x) * accel_scales_[0];
  hw_states_[1] = static_cast<double>(agmt.acc.axes.y) * accel_scales_[1];
  hw_states_[2] = static_cast<double>(agmt.acc.axes.z) * accel_scales_[2];
  hw_states_[3] = static_cast<double>(agmt.gyr.axes.x) * gyro_scales_[0];
  hw_states_[4] = static_cast<double>(agmt.gyr.axes.y) * gyro_scales_[1];
  hw_states_[5] = static_cast<double>(agmt.gyr.axes.z) * gyro_scales_[2];
  hw_states_[6] = static_cast<double>(agmt.mag.axes.x) * 0.15;
  hw_states_[7] = static_cast<double>(agmt.mag.axes.y) * 0.15;
  hw_states_[8] = static_cast<double>(agmt.mag.axes.z) * 0.15;

  return hardware_interface::return_type::OK;
}

} // namespace icm20948_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(icm20948_hardware::ICM20948Interface,
                       hardware_interface::SensorInterface)
