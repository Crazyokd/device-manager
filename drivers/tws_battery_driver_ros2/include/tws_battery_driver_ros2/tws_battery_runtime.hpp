#ifndef TWS_BATTERY_DRIVER_ROS2__TWS_BATTERY_RUNTIME_HPP_
#define TWS_BATTERY_DRIVER_ROS2__TWS_BATTERY_RUNTIME_HPP_

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "device_manager_core/device_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tws_battery_driver_ros2/msg/bms_status.hpp"
#include "tws_battery_driver_ros2/tws_battery_driver.hpp"

namespace tws_battery_driver_ros2
{

class TwsBatteryRuntime final : public device_manager::DeviceRuntimeBase
{
public:
  using LogHandler = std::function<void(const BatteryStatusLogRecord &)>;

  explicit TwsBatteryRuntime(
    BatteryDriverConfig config = {},
    TwsBatteryDriver::TransportFactory transport_factory = {});
  ~TwsBatteryRuntime() override;

  void initialize(const device_manager::DeviceDefinition & definition) override;
  device_manager::LifecycleState state() const override;
  std::vector<device_manager::TransitionRequest> parameter_transitions(
    device_manager::LifecycleState state,
    const device_manager::ParameterMap & parameters,
    const device_manager::ParameterMap &) override
  {
    return device_manager::standard_parameter_transitions(state, parameters);
  }
  device_manager::TransitionOutcome materialize(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome dematerialize() override;
  void set_event_handler(device_manager::EventHandler handler) override;
  void register_hook(device_manager::HookRegistrar registrar) override;

  bool poll_once();
  BatterySnapshot snapshot() const;
  int poll_interval_ms() const;
  void set_log_handler(LogHandler handler);

private:
  device_manager::TransitionOutcome configure(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome activate() override;
  device_manager::TransitionOutcome deactivate() override;
  device_manager::TransitionOutcome cleanup(
    const device_manager::ParameterMap & parameters) override;

  BatteryDriverConfig config_from(
    const device_manager::ParameterMap & parameters,
    std::string & error_message) const;
  void emit_event(device_manager::DeviceEvent event);
  void emit_error_event(
    std::string code,
    std::string message,
    device_manager::LifecycleState target_state);
  void emit_ok_event(std::string code, std::string message);
  void start_polling();
  void stop_polling();
  void poll_loop();
  void publish_snapshot(const BatterySnapshot & snapshot);
  sensor_msgs::msg::BatteryState build_battery_state_msg(
    const BatterySnapshot & snapshot) const;
  tws_battery_driver_ros2::msg::BmsStatus build_bms_status_msg(
    const BatterySnapshot & snapshot) const;

  mutable std::mutex mutex_;
  BatteryDriverConfig config_;
  TwsBatteryDriver::TransportFactory transport_factory_;
  device_manager::LifecycleState state_;
  device_manager::EventHandler event_handler_;
  LogHandler log_handler_;
  std::string last_poll_error_;
  std::shared_ptr<rclcpp::Node> output_node_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_publisher_;
  rclcpp::Publisher<tws_battery_driver_ros2::msg::BmsStatus>::SharedPtr bms_status_publisher_;
  std::mutex poll_mutex_;
  std::condition_variable poll_condition_;
  bool stop_polling_ {true};
  std::thread poll_thread_;
  std::unique_ptr<TwsBatteryDriver> driver_;
};

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__TWS_BATTERY_RUNTIME_HPP_
