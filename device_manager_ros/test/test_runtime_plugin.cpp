#include <utility>

#include "device_manager_core/device_manager.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace device_manager_ros
{

class TestRuntimePlugin final : public device_manager::IDeviceRuntime
{
public:
  void initialize(const device_manager::DeviceDefinition & definition) override
  {
    const auto marker = definition.parameters.find("marker");
    const bool complete_definition =
      definition.id == "plugin_device" && definition.type == "sensor" && definition.enabled &&
      marker != definition.parameters.end() &&
      std::get<std::string>(marker->second) == "loaded";
    const auto topology = definition.parameters.find("topology_marker");
    state_ = complete_definition ?
      device_manager::LifecycleState::kUnconfigured : device_manager::LifecycleState::kUnknown;
    if (complete_definition && topology != definition.parameters.end() &&
      std::get<std::string>(topology->second) == "from_yaml")
    {
      state_ = device_manager::LifecycleState::kInactive;
    }
  }

  device_manager::LifecycleState state() const override
  {
    return state_;
  }

  device_manager::TransitionOutcome request_transition(
    const device_manager::TransitionRequest &) override
  {
    return {device_manager::TransitionResult::kFailure, "test runtime has no transitions"};
  }

  std::vector<device_manager::TransitionRequest> parameter_transitions(
    device_manager::LifecycleState, const device_manager::ParameterMap &,
    const device_manager::ParameterMap &) override
  {
    return {};
  }

  void set_event_handler(device_manager::EventHandler) override {}
  void register_hook(device_manager::HookRegistrar) override {}

private:
  device_manager::LifecycleState state_{device_manager::LifecycleState::kUnknown};
};

}  // namespace device_manager_ros

PLUGINLIB_EXPORT_CLASS(
  device_manager_ros::TestRuntimePlugin,
  device_manager::IDeviceRuntime)
