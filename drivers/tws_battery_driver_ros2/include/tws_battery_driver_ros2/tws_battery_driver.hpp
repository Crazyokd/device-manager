#ifndef TWS_BATTERY_DRIVER_ROS2__TWS_BATTERY_DRIVER_HPP_
#define TWS_BATTERY_DRIVER_ROS2__TWS_BATTERY_DRIVER_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "tws_battery_driver_ros2/battery_snapshot.hpp"
#include "tws_battery_driver_ros2/battery_soc_transition_tracker.hpp"
#include "tws_battery_driver_ros2/battery_status_transition_log.hpp"
#include "tws_battery_driver_ros2/battery_status_transition_tracker.hpp"
#include "tws_battery_driver_ros2/battery_transport.hpp"
#include "tws_battery_driver_ros2/soc_spike_filter.hpp"

namespace tws_battery_driver_ros2
{

struct BatteryDriverConfig
{
  std::string can_interface {"can1"};
  uint32_t can_id {0x41U};
  bool use_extended_frame {true};
  int timeout_ms {600};
  int poll_interval_ms {1000};
  int identity_poll_divider {30};
  uint8_t device_address {0x01U};
};

struct BatteryPollResult
{
  bool ok {false};
  bool report_failure_event {false};
  std::string error_message;
  BatterySnapshot snapshot;
  std::vector<BatteryStatusLogRecord> log_records;
};

class TwsBatteryDriver
{
public:
  using TransportFactory =
    std::function<std::unique_ptr<BatteryTransport>(const BatteryDriverConfig &)>;

  explicit TwsBatteryDriver(
    BatteryDriverConfig config = {},
    TransportFactory transport_factory = {});

  bool configure(const BatteryDriverConfig & config, std::string & error_message);
  void cleanup();
  bool is_configured() const;

  BatteryPollResult poll_once();
  BatterySnapshot snapshot() const;
  const BatteryDriverConfig & config() const;

private:
  bool ensure_connected(std::string & error_message);
  bool read_fast_snapshot(BatterySnapshot & snapshot, std::string & error_message);
  void read_identity_snapshot(BatterySnapshot & snapshot, BatteryPollResult & result);
  BatteryPollResult make_poll_failure_result(const std::string & error_message);
  void append_status_log(
    const std::optional<BatteryStatusTransitionEvent> & event,
    BatteryPollResult & result) const;
  void append_soc_log(
    const std::optional<BatterySocTransitionEvent> & event,
    BatteryPollResult & result) const;

  BatteryDriverConfig config_;
  TransportFactory transport_factory_;
  std::unique_ptr<BatteryTransport> transport_;
  BatteryStatusTransitionTracker status_transition_tracker_;
  BatterySocTransitionTracker soc_transition_tracker_;
  SocSpikeFilter soc_filter_;
  BatterySnapshot last_snapshot_;
  bool has_successful_poll_ {false};
  std::size_t consecutive_poll_failures_ {0U};
  std::size_t poll_counter_ {0U};

  static constexpr std::size_t kConsecutivePollFailuresToOffline = 3U;
};

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__TWS_BATTERY_DRIVER_HPP_
