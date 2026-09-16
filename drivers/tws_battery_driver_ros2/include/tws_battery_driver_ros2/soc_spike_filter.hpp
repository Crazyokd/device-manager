#ifndef TWS_BATTERY_DRIVER_ROS2__SOC_SPIKE_FILTER_HPP_
#define TWS_BATTERY_DRIVER_ROS2__SOC_SPIKE_FILTER_HPP_

#include <cstddef>

namespace tws_battery_driver_ros2
{

class SocSpikeFilter
{
public:
  float filter(float raw_soc)
  {
    if (!has_accepted_soc_) {
      return accept(raw_soc);
    }

    if (raw_soc >= kFullSoc && accepted_soc_ < kNearFullSoc) {
      ++pending_full_soc_count_;
      if (pending_full_soc_count_ >= kFullSocConfirmCount) {
        return accept(raw_soc);
      }
      return accepted_soc_;
    }

    return accept(raw_soc);
  }

private:
  float accept(float soc)
  {
    has_accepted_soc_ = true;
    accepted_soc_ = soc;
    pending_full_soc_count_ = 0U;
    return accepted_soc_;
  }

  static constexpr float kFullSoc = 100.0F;
  static constexpr float kNearFullSoc = 99.0F;
  static constexpr std::size_t kFullSocConfirmCount = 3U;

  bool has_accepted_soc_ {false};
  float accepted_soc_ {0.0F};
  std::size_t pending_full_soc_count_ {0U};
};

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__SOC_SPIKE_FILTER_HPP_
