#ifndef TWS_BATTERY_DRIVER_ROS2__BATTERY_STATUS_TRANSITION_TRACKER_HPP_
#define TWS_BATTERY_DRIVER_ROS2__BATTERY_STATUS_TRANSITION_TRACKER_HPP_

#include <cstdint>
#include <optional>
#include <string>

namespace tws_battery_driver_ros2
{

enum class BatteryStatusTransitionType
{
  kOnline,
  kOffline,
  kWorkStateChanged,
};

struct BatteryStatusTransitionEvent
{
  BatteryStatusTransitionType type;
  uint16_t previous_work_state {0U};
  uint16_t current_work_state {0U};
  std::string error_message;
};

class BatteryStatusTransitionTracker
{
public:
  std::optional<BatteryStatusTransitionEvent> observe_online(uint16_t work_state)
  {
    if (!online_) {
      online_ = true;
      current_work_state_ = work_state;
      return BatteryStatusTransitionEvent {
        BatteryStatusTransitionType::kOnline,
        work_state,
        work_state,
        "",
      };
    }

    if (work_state == current_work_state_) {
      return std::nullopt;
    }

    const uint16_t previous_work_state = current_work_state_;
    current_work_state_ = work_state;
    return BatteryStatusTransitionEvent {
      BatteryStatusTransitionType::kWorkStateChanged,
      previous_work_state,
      work_state,
      "",
    };
  }

  std::optional<BatteryStatusTransitionEvent> observe_offline(
    const std::string & error_message)
  {
    if (!online_) {
      return std::nullopt;
    }

    online_ = false;
    return BatteryStatusTransitionEvent {
      BatteryStatusTransitionType::kOffline,
      current_work_state_,
      current_work_state_,
      error_message,
    };
  }

private:
  bool online_ {false};
  uint16_t current_work_state_ {0U};
};

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__BATTERY_STATUS_TRANSITION_TRACKER_HPP_
