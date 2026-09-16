#ifndef TWS_BATTERY_DRIVER_ROS2__BATTERY_SNAPSHOT_HPP_
#define TWS_BATTERY_DRIVER_ROS2__BATTERY_SNAPSHOT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace tws_battery_driver_ros2
{

struct BatterySnapshot
{
  bool connected {false};
  std::string last_error;
  uint16_t device_address {1U};
  uint16_t work_state {0U};
  float pack_voltage_v {0.0F};
  float pack_current_a {0.0F};
  float max_cell_voltage_v {0.0F};
  float min_cell_voltage_v {0.0F};
  float max_cell_temperature_c {0.0F};
  float min_cell_temperature_c {0.0F};
  float max_board_temperature_c {0.0F};
  float min_board_temperature_c {0.0F};
  float soc {0.0F};
  float soh {0.0F};
  float remain_capacity_mah {0.0F};
  uint32_t cycle_count {0U};
  uint32_t protect_status {0U};
  uint32_t io_status {0U};
  std::string serial_number;
  std::string software_version;
  std::string hardware_version;
};

std::string sanitize_identity_string(const std::string & raw_value);
float decode_temperature_c(uint16_t raw_value);
std::string version_to_text(uint16_t raw_value);
std::vector<std::string> decode_protection_bits(uint32_t mask);

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__BATTERY_SNAPSHOT_HPP_
