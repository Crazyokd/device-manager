#include "device_manager_ros/ros2_lifecycle_driver_adapter.hpp"

#include <future>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/key_value.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/parameter.hpp"

#include "ros2_runtime_utils.hpp"

namespace device_manager_ros
{
namespace
{

using device_manager::LifecycleState;
using device_manager::TransitionResult;

struct RecoveryHookState
{
  std::mutex mutex;
  std::optional<std::uint64_t> handled_event_revision;
};

LifecycleState lifecycle_state(std::uint8_t state)
{
  switch (state) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      return LifecycleState::kUnconfigured;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      return LifecycleState::kInactive;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      return LifecycleState::kActive;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
      return LifecycleState::kFinalized;
    case lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING:
      return LifecycleState::kConfiguring;
    case lifecycle_msgs::msg::State::TRANSITION_STATE_CLEANINGUP:
      return LifecycleState::kCleaningUp;
    case lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN:
      return LifecycleState::kShuttingDown;
    case lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING:
      return LifecycleState::kActivating;
    case lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING:
      return LifecycleState::kDeactivating;
    case lifecycle_msgs::msg::State::TRANSITION_STATE_ERRORPROCESSING:
      return LifecycleState::kErrorProcessing;
    default:
      return LifecycleState::kUnknown;
  }
}

std::optional<std::uint8_t> transition_id(LifecycleState from, LifecycleState target)
{
  using Transition = lifecycle_msgs::msg::Transition;
  if (from == LifecycleState::kUnconfigured && target == LifecycleState::kInactive) {
    return Transition::TRANSITION_CONFIGURE;
  }
  if (from == LifecycleState::kInactive && target == LifecycleState::kUnconfigured) {
    return Transition::TRANSITION_CLEANUP;
  }
  if (from == LifecycleState::kInactive && target == LifecycleState::kActive) {
    return Transition::TRANSITION_ACTIVATE;
  }
  if (from == LifecycleState::kActive && target == LifecycleState::kInactive) {
    return Transition::TRANSITION_DEACTIVATE;
  }
  if (target != LifecycleState::kFinalized) {
    return std::nullopt;
  }
  if (from == LifecycleState::kUnconfigured) {
    return Transition::TRANSITION_UNCONFIGURED_SHUTDOWN;
  }
  if (from == LifecycleState::kInactive) {
    return Transition::TRANSITION_INACTIVE_SHUTDOWN;
  }
  if (from == LifecycleState::kActive) {
    return Transition::TRANSITION_ACTIVE_SHUTDOWN;
  }
  return std::nullopt;
}

// 与 detail::transitions_to 有实质差异：不支持 kFinalized -> kUnconfigured
// （lifecycle 设备由外部进程托管，finalized 后不可由本 adapter 重新 materialize），
// 故保留本地拷贝，不并入共享实现。
std::vector<device_manager::TransitionRequest> transitions_to(
  LifecycleState from, LifecycleState target, const device_manager::ParameterMap & parameters)
{
  using Request = device_manager::TransitionRequest;
  if (from == target) {
    return {};
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
    if (target == LifecycleState::kUnconfigured || target == LifecycleState::kActive) {
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
        Request{LifecycleState::kInactive, target, {}}};
    }
  }
  return {};
}

rcl_interfaces::msg::Parameter parameter_message(
  const std::string & name, const device_manager::ParameterValue & value)
{
  return std::visit(
    [&name](const auto & typed_value) {
      return rclcpp::Parameter(name, typed_value).to_parameter_msg();
    }, value);
}

device_manager::Clock::time_point time_point(const builtin_interfaces::msg::Time & time)
{
  return device_manager::Clock::time_point{
    std::chrono::seconds(time.sec) + std::chrono::nanoseconds(time.nanosec)};
}

device_manager::DeviceEvent device_event(const device_manager_msgs::msg::DeviceEvent & message)
{
  device_manager::DeviceEvent result;
  result.timestamp = time_point(message.timestamp);
  result.level = static_cast<device_manager::EventLevel>(message.level);
  result.code = message.code;
  result.message = message.message;
  result.source = message.source;
  result.target_state = lifecycle_state(message.target_state);
  for (const auto & value : message.values) {
    result.values.insert_or_assign(value.key, value.value);
  }
  return result;
}

}  // namespace

Ros2LifecycleDriverAdapter::Ros2LifecycleDriverAdapter(
  rclcpp::Node & node, std::string node_name,
  std::chrono::milliseconds service_timeout)
: service_timeout_(service_timeout),
  state_cache_(std::make_shared<StateCache>()), event_sink_(std::make_shared<EventSink>())
{
  const auto prefix =
    detail::normalize_graph_name(std::move(node_name), "lifecycle node_name");
  get_state_client_ = node.create_client<lifecycle_msgs::srv::GetState>(prefix + "/get_state");
  change_state_client_ =
    node.create_client<lifecycle_msgs::srv::ChangeState>(prefix + "/change_state");
  set_parameters_client_ =
    node.create_client<rcl_interfaces::srv::SetParameters>(prefix + "/set_parameters");

  const std::weak_ptr<StateCache> weak_state = state_cache_;
  transition_subscription_ = node.create_subscription<lifecycle_msgs::msg::TransitionEvent>(
    prefix + "/transition_event", rclcpp::QoS(10),
    [weak_state](const lifecycle_msgs::msg::TransitionEvent & event) {
      if (const auto state = weak_state.lock()) {
        ++state->generation;
        state->value = static_cast<std::uint8_t>(lifecycle_state(event.goal_state.id));
      }
    });

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
          handler(device_event(event));
        }
      }
    });

  refresh_state();
}

LifecycleState Ros2LifecycleDriverAdapter::state() const
{
  refresh_state();
  return static_cast<LifecycleState>(state_cache_->value.load());
}

device_manager::TransitionOutcome Ros2LifecycleDriverAdapter::request_transition(
  const device_manager::TransitionRequest & request)
{
  std::lock_guard<std::mutex> lock(transition_mutex_);
  const auto id = transition_id(request.from_state, request.target_state);
  if (!id) {
    return {TransitionResult::kFailure, "unsupported lifecycle transition"};
  }

  if (!request.parameters.empty()) {
    if (request.from_state != LifecycleState::kUnconfigured ||
      request.target_state != LifecycleState::kInactive)
    {
      return {TransitionResult::kFailure, "parameters are only valid for configure"};
    }
    if (!set_parameters_client_->wait_for_service(service_timeout_)) {
      return {TransitionResult::kError, "set_parameters service unavailable"};
    }
    auto parameter_request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (const auto & parameter : request.parameters) {
      parameter_request->parameters.push_back(parameter_message(parameter.first, parameter.second));
    }
    auto future = set_parameters_client_->async_send_request(parameter_request);
    if (future.wait_for(service_timeout_) != std::future_status::ready) {
      set_parameters_client_->remove_pending_request(future);
      return {TransitionResult::kError, "set_parameters request timed out"};
    }
    const auto parameter_response = future.get();
    for (const auto & result : parameter_response->results) {
      if (!result.successful) {
        return {TransitionResult::kFailure, result.reason};
      }
    }
  }

  if (!change_state_client_->wait_for_service(service_timeout_)) {
    return {TransitionResult::kError, "change_state service unavailable"};
  }
  auto change_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  change_request->transition.id = *id;
  auto future = change_state_client_->async_send_request(change_request);
  if (future.wait_for(service_timeout_) != std::future_status::ready) {
    change_state_client_->remove_pending_request(future);
    return {TransitionResult::kError, "change_state request timed out"};
  }
  if (!future.get()->success) {
    return {TransitionResult::kFailure, "lifecycle transition rejected"};
  }
  if (!get_state_client_->wait_for_service(service_timeout_)) {
    return {TransitionResult::kError, "get_state service unavailable after transition"};
  }
  auto state_future = get_state_client_->async_send_request(
    std::make_shared<lifecycle_msgs::srv::GetState::Request>());
  if (state_future.wait_for(service_timeout_) != std::future_status::ready) {
    get_state_client_->remove_pending_request(state_future);
    return {TransitionResult::kError, "get_state request timed out after transition"};
  }
  const auto actual_state = lifecycle_state(state_future.get()->current_state.id);
  ++state_cache_->generation;
  state_cache_->value = static_cast<std::uint8_t>(actual_state);
  if (actual_state != request.target_state) {
    return {TransitionResult::kFailure, "lifecycle target state was not reached"};
  }
  return {TransitionResult::kSuccess, "lifecycle transition succeeded"};
}

void Ros2LifecycleDriverAdapter::set_event_handler(device_manager::EventHandler handler)
{
  std::lock_guard<std::mutex> lock(event_sink_->mutex);
  event_sink_->handler = std::move(handler);
}

void Ros2LifecycleDriverAdapter::register_hook(device_manager::HookRegistrar registrar)
{
  const auto recovery_state = std::make_shared<RecoveryHookState>();
  registrar(
    [recovery_state](const device_manager::HookContext & context) {
      if (!context.enabled) {
        return std::vector<device_manager::TransitionRequest>{};
      }

      if (context.latest_event &&
      context.latest_event->level >= device_manager::EventLevel::kError)
      {
        std::lock_guard<std::mutex> lock(recovery_state->mutex);
        const bool already_handled =
        recovery_state->handled_event_revision &&
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
      const auto automatic_target = context.state == LifecycleState::kUnconfigured ?
      LifecycleState::kInactive : LifecycleState::kActive;
      return transitions_to(context.state, automatic_target, context.parameters);
    });
}

void Ros2LifecycleDriverAdapter::refresh_state() const
{
  if (!get_state_client_->service_is_ready()) {
    return;
  }
  std::lock_guard<std::mutex> lock(refresh_mutex_);
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  if (state_cache_->query_in_flight) {
    if (now_ns < state_cache_->query_deadline_ns) {
      return;
    }
    const auto request_id = state_cache_->query_request_id.exchange(-1);
    if (request_id >= 0) {
      get_state_client_->remove_pending_request(request_id);
    }
    ++state_cache_->generation;
    state_cache_->query_in_flight = false;
  }

  state_cache_->query_in_flight = true;
  const auto token = ++state_cache_->query_token;
  state_cache_->active_query_token = token;
  state_cache_->query_deadline_ns = now_ns +
    std::chrono::duration_cast<std::chrono::nanoseconds>(service_timeout_).count();
  const std::weak_ptr<StateCache> weak_state = state_cache_;
  const auto generation = state_cache_->generation.load();
  try {
    const auto pending_request = get_state_client_->async_send_request(
      std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
      [weak_state, generation, token](
        rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
        if (const auto state = weak_state.lock()) {
          try {
            const auto response = future.get();
            if (state->generation == generation && state->active_query_token == token) {
              state->value = static_cast<std::uint8_t>(
                lifecycle_state(response->current_state.id));
            }
          } catch (...) {
          }
          if (state->active_query_token == token) {
            state->query_request_id = -1;
            state->query_in_flight = false;
          }
        }
      });
    if (state_cache_->active_query_token == token && state_cache_->query_in_flight) {
      state_cache_->query_request_id = pending_request.request_id;
    }
  } catch (...) {
    if (state_cache_->active_query_token == token) {
      state_cache_->query_request_id = -1;
      state_cache_->query_in_flight = false;
    }
  }
}

}  // namespace device_manager_ros
