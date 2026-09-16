#pragma once

// Test-only IDeviceRuntime implementation used by the core test-suite.
// Not part of the public API: production drivers implement IDeviceRuntime
// directly (see drivers/tws_battery_driver_ros2, drivers/ch020_imu_driver,
// drivers/smit_ros_driver).

#include <functional>
#include <mutex>

#include "device_manager_core/device_manager.hpp"

namespace device_manager
{

class InProcessDeviceDriver final : public IDeviceRuntime
{
public:
  using TransitionHandler = std::function<TransitionOutcome(const TransitionRequest &)>;

  explicit InProcessDeviceDriver(
    LifecycleState initial_state, TransitionHandler transition_handler = {},
    LifecycleHook hook = {});

  LifecycleState state() const override;
  TransitionOutcome request_transition(const TransitionRequest & request) override;
  std::vector<TransitionRequest> parameter_transitions(
    LifecycleState, const ParameterMap &, const ParameterMap &) override
  {
    return {};
  }
  void set_event_handler(EventHandler handler) override;
  void register_hook(HookRegistrar registrar) override;

  void emit_event(DeviceEvent event);

private:
  mutable std::mutex mutex_;
  std::mutex transition_mutex_;
  LifecycleState state_;
  TransitionHandler transition_handler_;
  LifecycleHook hook_;
  EventHandler event_handler_;
};

}  // namespace device_manager
