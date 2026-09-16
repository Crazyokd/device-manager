#ifndef ORADAR_ROS_DRIVER__ORADAR_LIDAR_RUNTIME_HPP_
#define ORADAR_ROS_DRIVER__ORADAR_LIDAR_RUNTIME_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "device_manager_core/device_manager.hpp"
#include "oradar_ros_driver/oradar_lidar_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace oradar_ros_driver
{

class OradarLidarRuntime final : public device_manager::DeviceRuntimeBase
{
public:
  using ScanHandler = std::function<void(const sensor_msgs::msg::LaserScan &)>;

  explicit OradarLidarRuntime(OradarLidarClientFactory client_factory = {});
  ~OradarLidarRuntime() override;

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

  void set_scan_handler(ScanHandler handler);
  OradarLidarConfig config() const;

private:
  device_manager::TransitionOutcome configure(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome activate() override;
  device_manager::TransitionOutcome deactivate() override;
  device_manager::TransitionOutcome cleanup(
    const device_manager::ParameterMap & parameters) override;

  OradarLidarConfig config_from(
    const device_manager::ParameterMap & parameters,
    std::string & error_message) const;
  bool connect_client(std::string & error_message);
  bool apply_device_settings(std::string & error_message);
  void read_loop();
  void stop_reader();
  sensor_msgs::msg::LaserScan to_scan_message(
    const ord_sdk::ScanFrameData & frame,
    const rclcpp::Time & stamp,
    double scan_time_seconds) const;
  void publish_scan(const sensor_msgs::msg::LaserScan & message);
  void emit_event(device_manager::DeviceEvent event);
  void emit_error_event(
    std::string code,
    std::string message,
    device_manager::LifecycleState target_state);
  void emit_ok_event(std::string code, std::string message);

  mutable std::mutex mutex_;
  OradarLidarConfig config_;
  OradarLidarClientFactory client_factory_;
  device_manager::LifecycleState state_;
  device_manager::EventHandler event_handler_;
  ScanHandler scan_handler_;
  std::shared_ptr<rclcpp::Node> output_node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  std::string topic_ {"scan"};
  std::string frame_id_ {"lidar_frame"};

  std::unique_ptr<OradarLidarClient> client_;
  std::thread read_thread_;
  bool read_thread_running_ {false};
  bool connected_event_emitted_ {false};
};

}  // namespace oradar_ros_driver

#endif  // ORADAR_ROS_DRIVER__ORADAR_LIDAR_RUNTIME_HPP_
