#ifndef PAGER100_LORA_BRIDGE__PAGER100_LORA_RUNTIME_HPP_
#define PAGER100_LORA_BRIDGE__PAGER100_LORA_RUNTIME_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "device_manager_core/device_manager.hpp"
#include "pager100_lora_bridge/pager100_lora_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace pager100_lora_bridge
{

class Pager100LoraRuntime final : public device_manager::DeviceRuntimeBase
{
public:
  explicit Pager100LoraRuntime(
    Pager100LoraConfig config = {},
    Pager100LoraDriver::TransportFactory transport_factory = {});
  ~Pager100LoraRuntime() override;

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

  void set_frame_handler(Pager100LoraDriver::FrameHandler handler);
  Pager100LoraSnapshot snapshot() const;

private:
  device_manager::TransitionOutcome configure(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome activate() override;
  device_manager::TransitionOutcome deactivate() override;
  device_manager::TransitionOutcome cleanup(
    const device_manager::ParameterMap & parameters) override;

  Pager100LoraConfig config_from(
    const device_manager::ParameterMap & parameters,
    std::string & error_message) const;
  void emit_event(device_manager::DeviceEvent event);
  void emit_error_event(
    std::string code,
    std::string message,
    device_manager::LifecycleState target_state);
  void emit_ok_event(std::string code, std::string message);
  void on_tx(const std_msgs::msg::UInt8MultiArray & message);
  void stop_output_spin();

  mutable std::mutex mutex_;
  Pager100LoraConfig config_;
  Pager100LoraDriver::TransportFactory transport_factory_;
  device_manager::LifecycleState state_;
  device_manager::EventHandler event_handler_;
  Pager100LoraDriver::FrameHandler frame_handler_;

  std::shared_ptr<rclcpp::Node> output_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> output_executor_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_subscription_;
  std::thread output_spin_thread_;

  // tx 回调来自 spin 线程，可能与 dematerialize 并发；shared_ptr 保证
  // 回调期间驱动对象存活。驱动先于 ROS 输出销毁，读线程不会比回调目标活得久。
  std::shared_ptr<Pager100LoraDriver> driver_;
};

}  // namespace pager100_lora_bridge

#endif  // PAGER100_LORA_BRIDGE__PAGER100_LORA_RUNTIME_HPP_
