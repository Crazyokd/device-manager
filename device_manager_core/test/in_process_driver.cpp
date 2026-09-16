#include "in_process_driver.hpp"

#include <exception>
#include <utility>

namespace device_manager
{
namespace
{

bool is_valid_transition(LifecycleState from, LifecycleState target)
{
  if (target == LifecycleState::kFinalized) {
    return from == LifecycleState::kUnconfigured || from == LifecycleState::kInactive ||
           from == LifecycleState::kActive;
  }
  if (from == LifecycleState::kFinalized && target == LifecycleState::kUnconfigured) {
    return true;
  }
  return
    (from == LifecycleState::kUnconfigured && target == LifecycleState::kInactive) ||
    (from == LifecycleState::kInactive && target == LifecycleState::kActive) ||
    (from == LifecycleState::kActive && target == LifecycleState::kInactive) ||
    (from == LifecycleState::kInactive && target == LifecycleState::kUnconfigured);
}

LifecycleState transition_state(LifecycleState from, LifecycleState target)
{
  if (target == LifecycleState::kFinalized) {
    return LifecycleState::kShuttingDown;
  }
  if (from == LifecycleState::kFinalized) {
    return LifecycleState::kConfiguring;
  }
  if (from == LifecycleState::kUnconfigured) {
    return LifecycleState::kConfiguring;
  }
  if (from == LifecycleState::kActive) {
    return LifecycleState::kDeactivating;
  }
  if (target == LifecycleState::kActive) {
    return LifecycleState::kActivating;
  }
  return LifecycleState::kCleaningUp;
}

}  // namespace

InProcessDeviceDriver::InProcessDeviceDriver(
  LifecycleState initial_state, TransitionHandler transition_handler, LifecycleHook hook)
: state_(initial_state), transition_handler_(std::move(transition_handler)), hook_(std::move(hook))
{
}

LifecycleState InProcessDeviceDriver::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

TransitionOutcome InProcessDeviceDriver::request_transition(const TransitionRequest & request)
{
  std::lock_guard<std::mutex> transition_lock(transition_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != request.from_state) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    if (!is_valid_transition(request.from_state, request.target_state)) {
      return {TransitionResult::kFailure, "unsupported lifecycle transition"};
    }
    state_ = transition_state(request.from_state, request.target_state);
  }

  TransitionOutcome outcome{TransitionResult::kSuccess, {}};
  if (transition_handler_) {
    try {
      outcome = transition_handler_(request);
    } catch (const std::exception & exception) {
      outcome = {TransitionResult::kError, exception.what()};
    } catch (...) {
      outcome = {TransitionResult::kError, "transition handler threw an unknown exception"};
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = outcome.result == TransitionResult::kSuccess ?
      request.target_state : request.from_state;
  }
  return outcome;
}

void InProcessDeviceDriver::set_event_handler(EventHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  event_handler_ = std::move(handler);
}

void InProcessDeviceDriver::register_hook(HookRegistrar registrar)
{
  LifecycleHook hook;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hook = hook_;
  }
  if (hook) {
    registrar(std::move(hook));
  }
}

void InProcessDeviceDriver::emit_event(DeviceEvent event)
{
  EventHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handler = event_handler_;
  }
  if (handler) {
    handler(std::move(event));
  }
}

}  // namespace device_manager
