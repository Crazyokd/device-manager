#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "device_manager_core/device_manager.hpp"
#include "device_manager_msgs/msg/device_event.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

namespace device_manager_ros
{

class Ros2LifecycleDriverAdapter final : public device_manager::IDeviceRuntime
{
public:
  explicit Ros2LifecycleDriverAdapter(
    rclcpp::Node & node, std::string node_name,
    std::chrono::milliseconds service_timeout = std::chrono::seconds(1));

  device_manager::LifecycleState state() const override;
  device_manager::TransitionOutcome request_transition(
    const device_manager::TransitionRequest & request) override;
  std::vector<device_manager::TransitionRequest> parameter_transitions(
    device_manager::LifecycleState state,
    const device_manager::ParameterMap & parameters,
    const device_manager::ParameterMap &) override
  {
    return device_manager::standard_parameter_transitions(state, parameters);
  }
  void set_event_handler(device_manager::EventHandler handler) override;
  void register_hook(device_manager::HookRegistrar registrar) override;

private:
  struct StateCache
  {
    std::atomic<std::uint8_t> value{
      static_cast<std::uint8_t>(device_manager::LifecycleState::kUnknown)};
    std::atomic<std::uint64_t> generation{0};
    std::atomic<bool> query_in_flight{false};
    std::atomic<std::uint64_t> query_token{0};
    std::atomic<std::uint64_t> active_query_token{0};
    std::atomic<std::int64_t> query_deadline_ns{0};
    std::atomic<std::int64_t> query_request_id{-1};
  };

  struct EventSink
  {
    std::mutex mutex;
    device_manager::EventHandler handler;
  };

  void refresh_state() const;

  std::chrono::milliseconds service_timeout_;
  std::shared_ptr<StateCache> state_cache_;
  std::shared_ptr<EventSink> event_sink_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_client_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr
    transition_subscription_;
  rclcpp::Subscription<device_manager_msgs::msg::DeviceEvent>::SharedPtr event_subscription_;
  std::mutex transition_mutex_;
  mutable std::mutex refresh_mutex_;
};

}  // namespace device_manager_ros
