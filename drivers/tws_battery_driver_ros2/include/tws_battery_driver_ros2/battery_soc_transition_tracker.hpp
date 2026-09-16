#ifndef TWS_BATTERY_DRIVER_ROS2__BATTERY_SOC_TRANSITION_TRACKER_HPP_
#define TWS_BATTERY_DRIVER_ROS2__BATTERY_SOC_TRANSITION_TRACKER_HPP_

#include <optional>

namespace tws_battery_driver_ros2
{

struct BatterySocTransitionEvent
{
  float previous_soc;
  float current_soc;
};

class BatterySocTransitionTracker
{
public:
  std::optional<BatterySocTransitionEvent> observe(float soc)
  {
    if (!current_soc_.has_value()) {
      current_soc_ = soc;
      return std::nullopt;
    }
    if (soc == *current_soc_) {
      return std::nullopt;
    }

    const float previous_soc = *current_soc_;
    current_soc_ = soc;
    return BatterySocTransitionEvent {previous_soc, soc};
  }

private:
  std::optional<float> current_soc_;
};

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__BATTERY_SOC_TRANSITION_TRACKER_HPP_
