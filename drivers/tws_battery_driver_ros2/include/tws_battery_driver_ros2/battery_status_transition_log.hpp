#ifndef TWS_BATTERY_DRIVER_ROS2__BATTERY_STATUS_TRANSITION_LOG_HPP_
#define TWS_BATTERY_DRIVER_ROS2__BATTERY_STATUS_TRANSITION_LOG_HPP_

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include "tws_battery_driver_ros2/battery_soc_transition_tracker.hpp"
#include "tws_battery_driver_ros2/battery_status_transition_tracker.hpp"

namespace tws_battery_driver_ros2
{

enum class BatteryStatusLogLevel
{
  kInfo,
  kWarn,
};

struct BatteryStatusLogRecord
{
  BatteryStatusLogLevel level;
  std::string message;
};

inline std::string battery_work_state_to_text(uint16_t work_state)
{
  switch (work_state) {
    case 0U:
      return "EM_IDLE";
    case 1U:
      return "EM_CHARGE";
    case 2U:
      return "EM_DISCHARGE";
    case 3U:
      return "EM_SLEEP";
    case 4U:
      return "EM_PROTECT";
    case 5U:
      return "EM_SHUTDOWN";
    default:
      return "UNKNOWN";
  }
}

inline std::string format_work_state(uint16_t work_state)
{
  return battery_work_state_to_text(work_state) + "(" +
         std::to_string(work_state) + ")";
}

inline std::string format_soc(float soc)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(1) << soc << '%';
  return stream.str();
}

inline BatteryStatusLogRecord make_battery_soc_log_record(
  const BatterySocTransitionEvent & event)
{
  return BatteryStatusLogRecord {
    BatteryStatusLogLevel::kInfo,
    "电池电量变化: " + format_soc(event.previous_soc) +
    " -> " + format_soc(event.current_soc),
  };
}

inline BatteryStatusLogRecord make_battery_status_log_record(
  const BatteryStatusTransitionEvent & event)
{
  switch (event.type) {
    case BatteryStatusTransitionType::kOnline:
      return BatteryStatusLogRecord {
        BatteryStatusLogLevel::kInfo,
        "电池上线: work_state=" + format_work_state(event.current_work_state),
      };
    case BatteryStatusTransitionType::kOffline:
      return BatteryStatusLogRecord {
        BatteryStatusLogLevel::kWarn,
        "电池掉线: " + event.error_message,
      };
    case BatteryStatusTransitionType::kWorkStateChanged:
      return BatteryStatusLogRecord {
        BatteryStatusLogLevel::kInfo,
        "电池工作状态变化: " + format_work_state(event.previous_work_state) +
        " -> " + format_work_state(event.current_work_state),
      };
  }

  return BatteryStatusLogRecord {BatteryStatusLogLevel::kInfo, ""};
}

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__BATTERY_STATUS_TRANSITION_LOG_HPP_
