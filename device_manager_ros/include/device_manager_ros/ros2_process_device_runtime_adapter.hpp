#pragma once

#include <chrono>
#include <memory>
#include <map>
#include <mutex>
#include <string>

#include <sys/types.h>

#include "device_manager_core/device_manager.hpp"
#include "device_manager_msgs/msg/device_event.hpp"
#include "rclcpp/rclcpp.hpp"

namespace device_manager_ros
{

class Ros2ProcessDeviceRuntimeAdapter final : public device_manager::DeviceRuntimeBase
{
public:
  struct ProcessSpec
  {
    std::string package;
    std::string executable;
    std::string node_name;
    std::string namespace_;  // 可选 ROS namespace；非空时 spawn 加 `-r __ns:=`
    std::map<std::string, std::string> parameter_mappings;
    device_manager::ParameterMap topology_parameters;
  };

  Ros2ProcessDeviceRuntimeAdapter(
    rclcpp::Node & node, ProcessSpec process,
    std::chrono::milliseconds service_timeout = std::chrono::seconds(1));
  ~Ros2ProcessDeviceRuntimeAdapter() override;

  device_manager::LifecycleState state() const override;
  std::vector<device_manager::TransitionRequest> parameter_transitions(
    device_manager::LifecycleState state,
    const device_manager::ParameterMap & parameters,
    const device_manager::ParameterMap &) override
  {
    return device_manager::standard_parameter_transitions(state, parameters);
  }
  void set_event_handler(device_manager::EventHandler handler) override;
  void register_hook(device_manager::HookRegistrar registrar) override;

protected:
  device_manager::TransitionOutcome materialize(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome configure(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome activate() override;
  device_manager::TransitionOutcome deactivate() override;
  device_manager::TransitionOutcome cleanup(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome dematerialize() override;

private:
  struct EventSink
  {
    std::mutex mutex;
    device_manager::EventHandler handler;
  };

  void emit_process_exited_event(int status) const;
  void reap_child_locked() const;
  device_manager::TransitionOutcome start_process_locked(
    const device_manager::ParameterMap & parameters);
  device_manager::TransitionOutcome stop_process_locked();

  ProcessSpec process_;
  std::chrono::milliseconds service_timeout_;
  std::shared_ptr<EventSink> event_sink_;
  rclcpp::Subscription<device_manager_msgs::msg::DeviceEvent>::SharedPtr event_subscription_;
  mutable std::mutex state_mutex_;
  mutable pid_t child_pid_{-1};
  device_manager::ParameterMap materialized_parameters_;
  device_manager::LifecycleState cached_state_{device_manager::LifecycleState::kFinalized};
};

}  // namespace device_manager_ros
