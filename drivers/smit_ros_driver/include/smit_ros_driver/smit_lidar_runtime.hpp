#ifndef SMIT_ROS_DRIVER__SMIT_LIDAR_RUNTIME_HPP_
#define SMIT_ROS_DRIVER__SMIT_LIDAR_RUNTIME_HPP_

#include <memory>
#include <mutex>

#include "device_manager_core/device_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "smit_ros_driver/smit_lidar_driver.hpp"

namespace smit_ros_driver
{

class SmitLidarRuntime final : public device_manager::DeviceRuntimeBase
{
public:
  explicit SmitLidarRuntime(
    SmitLidarConfig config = {},
    SmitLidarDriver::TransportFactory transport_factory = {});

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

  void set_scan_handler(SmitLidarDriver::ScanHandler handler);
  void set_config_handler(SmitLidarDriver::ConfigHandler handler);
  SmitLidarSnapshot snapshot() const;

private:
  device_manager::TransitionOutcome configure(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome activate() override;
  device_manager::TransitionOutcome deactivate() override;
  device_manager::TransitionOutcome cleanup(
    const device_manager::ParameterMap & parameters) override;

  SmitLidarConfig config_from(
    const device_manager::ParameterMap & parameters,
    std::string & error_message) const;
  void emit_event(device_manager::DeviceEvent event);
  void emit_error_event(
    std::string code,
    std::string message,
    device_manager::LifecycleState target_state);
  void emit_ok_event(std::string code, std::string message);
  void publish_scan(const SmitScan & scan);

  mutable std::mutex mutex_;
  SmitLidarConfig config_;
  SmitLidarDriver::TransportFactory transport_factory_;
  device_manager::LifecycleState state_;
  device_manager::EventHandler event_handler_;
  SmitLidarDriver::ScanHandler scan_handler_;
  SmitLidarDriver::ConfigHandler config_handler_;
  std::shared_ptr<rclcpp::Node> output_node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  std::string topic_ {"scan"};
  std::string frame_id_ {"lidar_frame"};

  // Destroy the driver first so its read thread cannot outlive callback targets.
  std::unique_ptr<SmitLidarDriver> driver_;
};

}  // namespace smit_ros_driver

#endif  // SMIT_ROS_DRIVER__SMIT_LIDAR_RUNTIME_HPP_
