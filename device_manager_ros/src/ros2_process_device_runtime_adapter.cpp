#include "device_manager_ros/ros2_process_device_runtime_adapter.hpp"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <spawn.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include "diagnostic_msgs/msg/key_value.hpp"
#include "yaml-cpp/yaml.h"

#include "ros2_runtime_utils.hpp"

extern char ** environ;

namespace device_manager_ros
{
namespace
{

using device_manager::LifecycleState;
using device_manager::ParameterMap;
using device_manager::ParameterValue;
using device_manager::TransitionRequest;
using device_manager::TransitionResult;

struct RecoveryHookState
{
  std::mutex mutex;
  std::optional<std::uint64_t> handled_event_revision;
};

// process.namespace_ 允许为空（映射到 "/"），与 detail::normalize_graph_name
// 空名抛异常的语义不同；非空时委托共享实现。
std::string normalize_graph_name(std::string name)
{
  if (name.empty()) {return "/";}
  return detail::normalize_graph_name(std::move(name), "process.namespace_");
}

using detail::to_core_event;

using detail::transitions_to;

std::string scalar_to_string(bool value)
{
  return value ? "true" : "false";
}

std::string scalar_to_string(std::int64_t value)
{
  return std::to_string(value);
}

std::string scalar_to_string(double value)
{
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

std::string scalar_to_string(const std::string & value)
{
  YAML::Emitter emitter;
  emitter << YAML::DoubleQuoted << value;
  return emitter.c_str();
}

template<typename T>
std::string vector_to_string(const std::vector<T> & values)
{
  std::ostringstream stream;
  stream << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0U) {
      stream << ',';
    }
    stream << scalar_to_string(values[index]);
  }
  stream << ']';
  return stream.str();
}

std::string parameter_value_to_string(const ParameterValue & value)
{
  return std::visit(
    [](const auto & typed_value) -> std::string {
      using Value = std::decay_t<decltype(typed_value)>;
      if constexpr (std::is_same_v<Value, std::vector<bool>>||
      std::is_same_v<Value, std::vector<std::int64_t>>||
      std::is_same_v<Value, std::vector<double>>||
      std::is_same_v<Value, std::vector<std::string>>)
      {
        return vector_to_string(typed_value);
      } else {
        return scalar_to_string(typed_value);
      }
    },
    value);
}

}  // namespace

Ros2ProcessDeviceRuntimeAdapter::Ros2ProcessDeviceRuntimeAdapter(
  rclcpp::Node & node, ProcessSpec process,
  std::chrono::milliseconds service_timeout)
: process_(std::move(process)),
  service_timeout_(service_timeout),
  event_sink_(std::make_shared<EventSink>())
{
  if (process_.package.empty()) {
    throw std::invalid_argument("process.package must not be empty");
  }
  if (process_.executable.empty()) {
    throw std::invalid_argument("process.executable must not be empty");
  }
  if (process_.node_name.empty()) {
    throw std::invalid_argument("process.node_name must not be empty");
  }
  if (service_timeout_ <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("service_timeout must be positive");
  }
  for (const auto & [target, source] : process_.parameter_mappings) {
    (void)source;
    if (process_.topology_parameters.find(target) != process_.topology_parameters.end()) {
      throw std::invalid_argument("topology parameter conflicts with mapped parameter: " + target);
    }
  }
  if (prctl(PR_SET_CHILD_SUBREAPER, 1) != 0) {
    throw std::runtime_error(
            std::string{"failed to register ROS process subreaper: "} + std::strerror(errno));
  }

  auto prefix = normalize_graph_name(process_.namespace_);
  if (prefix != "/") {
    prefix += '/';
  }
  prefix += process_.node_name;
  const std::weak_ptr<EventSink> weak_sink = event_sink_;
  event_subscription_ = node.create_subscription<device_manager_msgs::msg::DeviceEvent>(
    prefix + "/device_event", rclcpp::QoS(10),
    [weak_sink](const device_manager_msgs::msg::DeviceEvent & event) {
      if (const auto sink = weak_sink.lock()) {
        device_manager::EventHandler handler;
        {
          std::lock_guard<std::mutex> lock(sink->mutex);
          handler = sink->handler;
        }
        if (handler) {
          handler(to_core_event(event));
        }
      }
    });
}

Ros2ProcessDeviceRuntimeAdapter::~Ros2ProcessDeviceRuntimeAdapter()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  (void)stop_process_locked();
}

LifecycleState Ros2ProcessDeviceRuntimeAdapter::state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  reap_child_locked();
  return cached_state_;
}

device_manager::TransitionOutcome Ros2ProcessDeviceRuntimeAdapter::materialize(
  const ParameterMap & parameters)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  reap_child_locked();
  if (child_pid_ > 0) {
    cached_state_ = LifecycleState::kUnconfigured;
    return {TransitionResult::kSuccess, "ROS process already materialized"};
  }

  return start_process_locked(parameters);
}

device_manager::TransitionOutcome Ros2ProcessDeviceRuntimeAdapter::start_process_locked(
  const ParameterMap & parameters)
{
  ParameterMap child_parameters = process_.topology_parameters;
  for (const auto & [target, source] : process_.parameter_mappings) {
    const auto value = parameters.find(source);
    if (value == parameters.end()) {
      return {TransitionResult::kError, "missing mapped device parameter: " + source};
    }
    child_parameters.insert_or_assign(target, value->second);
  }

  std::vector<std::string> arguments{
    "ros2", "run", process_.package, process_.executable, "--ros-args",
    "-r", "__node:=" + process_.node_name};
  if (!process_.namespace_.empty()) {
    arguments.push_back("-r");
    arguments.push_back("__ns:=" + process_.namespace_);
  }
  for (const auto & [name, value] : child_parameters) {
    arguments.push_back("-p");
    arguments.push_back(name + ":=" + parameter_value_to_string(value));
  }

  std::vector<char *> argv;
  argv.reserve(arguments.size() + 1U);
  for (auto & argument : arguments) {
    argv.push_back(argument.data());
  }
  argv.push_back(nullptr);

  posix_spawnattr_t spawn_attributes;
  posix_spawnattr_init(&spawn_attributes);
  posix_spawnattr_setflags(&spawn_attributes, POSIX_SPAWN_SETPGROUP);
  posix_spawnattr_setpgroup(&spawn_attributes, 0);
  pid_t child_pid = -1;
  const auto spawn_result =
    posix_spawnp(&child_pid, "ros2", nullptr, &spawn_attributes, argv.data(), environ);
  posix_spawnattr_destroy(&spawn_attributes);
  if (spawn_result != 0) {
    return {TransitionResult::kError, std::string{"failed to start ROS process: "} +
      std::strerror(spawn_result)};
  }
  child_pid_ = child_pid;
  materialized_parameters_ = parameters;
  cached_state_ = LifecycleState::kUnconfigured;
  return {TransitionResult::kSuccess, "ROS process materialized"};
}

device_manager::TransitionOutcome Ros2ProcessDeviceRuntimeAdapter::configure(
  const ParameterMap & parameters)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  reap_child_locked();
  if (child_pid_ > 0 && !parameters.empty() && parameters != materialized_parameters_) {
    const auto stopped = stop_process_locked();
    if (stopped.result != TransitionResult::kSuccess) {
      return stopped;
    }
  }
  if (child_pid_ <= 0) {
    const auto started = start_process_locked(parameters);
    if (started.result != TransitionResult::kSuccess) {
      return started;
    }
  }
  cached_state_ = LifecycleState::kInactive;
  return {TransitionResult::kSuccess, "ROS process configured"};
}

device_manager::TransitionOutcome Ros2ProcessDeviceRuntimeAdapter::activate()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  reap_child_locked();
  if (child_pid_ <= 0) {
    return {TransitionResult::kError, "ROS process is not running"};
  }
  cached_state_ = LifecycleState::kActive;
  return {TransitionResult::kSuccess, "ROS process is active"};
}

device_manager::TransitionOutcome Ros2ProcessDeviceRuntimeAdapter::deactivate()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  reap_child_locked();
  if (child_pid_ <= 0) {
    return {TransitionResult::kError, "ROS process is not running"};
  }
  cached_state_ = LifecycleState::kInactive;
  return {TransitionResult::kSuccess, "ROS process deactivated"};
}

device_manager::TransitionOutcome Ros2ProcessDeviceRuntimeAdapter::cleanup(
  const ParameterMap &)
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  const auto stopped = stop_process_locked();
  if (stopped.result != TransitionResult::kSuccess) {
    return stopped;
  }
  cached_state_ = LifecycleState::kUnconfigured;
  return {TransitionResult::kSuccess, "ROS process cleaned up"};
}

device_manager::TransitionOutcome Ros2ProcessDeviceRuntimeAdapter::dematerialize()
{
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  return stop_process_locked();
}

void Ros2ProcessDeviceRuntimeAdapter::set_event_handler(
  device_manager::EventHandler handler)
{
  std::lock_guard<std::mutex> lock(event_sink_->mutex);
  event_sink_->handler = std::move(handler);
}

void Ros2ProcessDeviceRuntimeAdapter::register_hook(
  device_manager::HookRegistrar registrar)
{
  const auto recovery_state = std::make_shared<RecoveryHookState>();
  registrar(
    [recovery_state](const device_manager::HookContext & context) {
      if (context.latest_event && context.latest_event_revision != 0U) {
        std::lock_guard<std::mutex> lock(recovery_state->mutex);
        const bool already_handled = recovery_state->handled_event_revision &&
        *recovery_state->handled_event_revision >= context.latest_event_revision;
        const bool target_reached = context.state == context.latest_event->target_state;
        if (!already_handled && !target_reached) {
          auto requests = transitions_to(
            context.state, context.latest_event->target_state, context.parameters);
          if (!requests.empty()) {
            return requests;
          }
        }
        if (!already_handled) {
          recovery_state->handled_event_revision = context.latest_event_revision;
        }
      }

      if (context.state == LifecycleState::kFinalized) {
        if (!context.enabled) {
          return std::vector<TransitionRequest>{};
        }
        return transitions_to(context.state, LifecycleState::kUnconfigured, context.parameters);
      }

      if (!context.enabled) {
        return transitions_to(context.state, LifecycleState::kFinalized, context.parameters);
      }

      const auto automatic_target = context.state == LifecycleState::kUnconfigured ?
      LifecycleState::kInactive : LifecycleState::kActive;
      return transitions_to(context.state, automatic_target, context.parameters);
    });
}

void Ros2ProcessDeviceRuntimeAdapter::emit_process_exited_event(int status) const
{
  device_manager::EventHandler handler;
  {
    std::lock_guard<std::mutex> lock(event_sink_->mutex);
    handler = event_sink_->handler;
  }
  if (!handler) {
    return;
  }
  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = device_manager::EventLevel::kError;
  event.code = "ros2_process.process_exited";
  event.message = "managed ROS process exited";
  event.source = "ros2_process";
  event.target_state = LifecycleState::kFinalized;
  event.values.insert_or_assign("status", std::to_string(status));
  handler(std::move(event));
}

void Ros2ProcessDeviceRuntimeAdapter::reap_child_locked() const
{
  if (child_pid_ <= 0) {
    return;
  }
  int status = 0;
  const auto result = waitpid(child_pid_, &status, WNOHANG);
  if (result == child_pid_) {
    const auto process_group = child_pid_;
    (void)kill(-process_group, SIGKILL);
    const auto deadline = std::chrono::steady_clock::now() + service_timeout_;
    std::optional<std::chrono::steady_clock::time_point> group_gone_since;
    while (std::chrono::steady_clock::now() < deadline) {
      int descendant_status = 0;
      while (waitpid(-process_group, &descendant_status, WNOHANG) > 0) {}
      if (kill(-process_group, 0) != 0 && errno == ESRCH) {
        const auto now = std::chrono::steady_clock::now();
        if (!group_gone_since) {
          group_gone_since = now;
        } else if (now - *group_gone_since >= std::chrono::milliseconds(100)) {
          break;
        }
      } else {
        group_gone_since.reset();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    child_pid_ = -1;
    emit_process_exited_event(status);
  }
}

device_manager::TransitionOutcome Ros2ProcessDeviceRuntimeAdapter::stop_process_locked()
{
  if (child_pid_ <= 0) {
    cached_state_ = LifecycleState::kFinalized;
    return {TransitionResult::kSuccess, "ROS process finalized"};
  }

  const auto pid = child_pid_;
  if (kill(-pid, SIGTERM) != 0 && errno != ESRCH) {
    return {TransitionResult::kError, std::string{"failed to stop ROS process: "} +
      std::strerror(errno)};
  }

  const auto deadline = std::chrono::steady_clock::now() + service_timeout_;
  int status = 0;
  const auto reap_group = [&]() {
      while (waitpid(-pid, &status, WNOHANG) > 0) {}
    };
  while (std::chrono::steady_clock::now() < deadline) {
    reap_group();
    if (kill(-pid, 0) != 0 && errno == ESRCH) {
      child_pid_ = -1;
      cached_state_ = LifecycleState::kFinalized;
      return {TransitionResult::kSuccess, "ROS process finalized"};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (kill(-pid, SIGKILL) != 0 && errno != ESRCH) {
    return {TransitionResult::kError, std::string{"failed to kill ROS process: "} +
      std::strerror(errno)};
  }
  const auto kill_deadline = std::chrono::steady_clock::now() + service_timeout_;
  while (std::chrono::steady_clock::now() < kill_deadline) {
    reap_group();
    if (kill(-pid, 0) != 0 && errno == ESRCH) {
      child_pid_ = -1;
      cached_state_ = LifecycleState::kFinalized;
      return {TransitionResult::kSuccess, "ROS process finalized"};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return {TransitionResult::kError, "managed ROS process group did not exit"};
}

}  // namespace device_manager_ros
