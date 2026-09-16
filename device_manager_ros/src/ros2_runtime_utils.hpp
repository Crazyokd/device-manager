#pragma once

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "device_manager_core/device_manager.hpp"
#include "device_manager_msgs/msg/device_event.hpp"

namespace device_manager_ros
{
namespace detail
{

// 规范化 ROS graph 名称（节点 / topic / service 通用规则）：补前导 '/'、去尾部 '/'；
// 空名抛 invalid_argument，label 用于错误文案。
inline std::string normalize_graph_name(std::string name, const std::string & label)
{
  if (name.empty()) {
    throw std::invalid_argument(label + " must not be empty");
  }
  if (name.front() != '/') {
    name.insert(name.begin(), '/');
  }
  while (name.size() > 1U && name.back() == '/') {
    name.pop_back();
  }
  return name;
}

// ROS DeviceEvent 消息转核心事件（device_event topic 契约的共享转换）。
inline device_manager::Clock::time_point time_point(const builtin_interfaces::msg::Time & time)
{
  return device_manager::Clock::time_point{
    std::chrono::seconds(time.sec) + std::chrono::nanoseconds(time.nanosec)};
}

inline device_manager::DeviceEvent to_core_event(
  const device_manager_msgs::msg::DeviceEvent & message)
{
  device_manager::DeviceEvent result;
  result.timestamp = time_point(message.timestamp);
  result.level = static_cast<device_manager::EventLevel>(message.level);
  result.code = message.code;
  result.message = message.message;
  result.source = message.source;
  result.target_state = static_cast<device_manager::LifecycleState>(message.target_state);
  for (const auto & value : message.values) {
    result.values.insert_or_assign(value.key, value.value);
  }
  return result;
}

// 状态机自动推进路径；cleanup_parameters 用于回到 kUnconfigured 的 cleanup 过渡。
inline std::vector<device_manager::TransitionRequest> transitions_to(
  device_manager::LifecycleState from, device_manager::LifecycleState target,
  const device_manager::ParameterMap & parameters,
  const device_manager::ParameterMap & cleanup_parameters = {})
{
  using device_manager::LifecycleState;
  using Request = device_manager::TransitionRequest;
  if (from == target) {
    return {};
  }
  if (from == LifecycleState::kFinalized && target == LifecycleState::kUnconfigured) {
    return {Request{from, target, parameters}};
  }
  if (target == LifecycleState::kFinalized &&
    (from == LifecycleState::kUnconfigured || from == LifecycleState::kInactive ||
    from == LifecycleState::kActive))
  {
    return {Request{from, target, {}}};
  }
  if (from == LifecycleState::kUnconfigured) {
    if (target == LifecycleState::kInactive) {
      return {Request{from, target, parameters}};
    }
    if (target == LifecycleState::kActive) {
      return {
        Request{from, LifecycleState::kInactive, parameters},
        Request{LifecycleState::kInactive, target, {}}};
    }
  }
  if (from == LifecycleState::kInactive) {
    if (target == LifecycleState::kUnconfigured) {
      return {Request{from, target, cleanup_parameters}};
    }
    if (target == LifecycleState::kActive) {
      return {Request{from, target, {}}};
    }
  }
  if (from == LifecycleState::kActive) {
    if (target == LifecycleState::kInactive) {
      return {Request{from, target, {}}};
    }
    if (target == LifecycleState::kUnconfigured) {
      return {
        Request{from, LifecycleState::kInactive, {}},
        Request{LifecycleState::kInactive, target, cleanup_parameters}};
    }
  }
  return {};
}

}  // namespace detail
}  // namespace device_manager_ros
