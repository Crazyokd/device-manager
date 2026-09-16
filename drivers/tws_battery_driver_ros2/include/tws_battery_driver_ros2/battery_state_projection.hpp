#ifndef TWS_BATTERY_DRIVER_ROS2__BATTERY_STATE_PROJECTION_HPP_
#define TWS_BATTERY_DRIVER_ROS2__BATTERY_STATE_PROJECTION_HPP_

#include <cstdint>

#include "sensor_msgs/msg/battery_state.hpp"

namespace tws_battery_driver_ros2
{

struct BatteryStateProjection
{
  uint16_t work_state;
  uint8_t power_supply_status;
};

inline BatteryStateProjection project_battery_state(
  bool connected, uint16_t work_state, float soc)
{
  if (!connected) {
    return {
      0U,
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING,
    };
  }

  uint8_t power_supply_status =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  if (work_state == 1U) {
    power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else if (work_state == 2U) {
    power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  } else if (soc >= 99.0F) {
    power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  }

  return {work_state, power_supply_status};
}

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__BATTERY_STATE_PROJECTION_HPP_
