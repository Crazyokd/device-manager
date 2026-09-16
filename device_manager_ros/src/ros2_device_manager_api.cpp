#include "device_manager_ros/ros2_device_manager_api.hpp"

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "device_manager_msgs/msg/device_event.hpp"
#include "device_manager_msgs/msg/device_state.hpp"
#include "device_manager_msgs/msg/transition_record.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp/parameter.hpp"

namespace device_manager_ros
{
namespace
{

std::optional<device_manager::LifecycleState> lifecycle_state(std::uint8_t value)
{
  switch (value) {
    case 0:
      return device_manager::LifecycleState::kUnknown;
    case 1:
      return device_manager::LifecycleState::kUnconfigured;
    case 2:
      return device_manager::LifecycleState::kInactive;
    case 3:
      return device_manager::LifecycleState::kActive;
    case 4:
      return device_manager::LifecycleState::kFinalized;
    case 10:
      return device_manager::LifecycleState::kConfiguring;
    case 11:
      return device_manager::LifecycleState::kCleaningUp;
    case 12:
      return device_manager::LifecycleState::kShuttingDown;
    case 13:
      return device_manager::LifecycleState::kActivating;
    case 14:
      return device_manager::LifecycleState::kDeactivating;
    case 15:
      return device_manager::LifecycleState::kErrorProcessing;
    default:
      return std::nullopt;
  }
}

std::optional<device_manager::RequestPriority> priority(std::uint8_t value)
{
  if (value == 0U) {
    return device_manager::RequestPriority::kNormal;
  }
  if (value == 1U) {
    return device_manager::RequestPriority::kUrgent;
  }
  return std::nullopt;
}

device_manager::ParameterValue parameter_value(
  const rcl_interfaces::msg::Parameter & message)
{
  const auto parameter = rclcpp::Parameter::from_parameter_msg(message);
  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return parameter.as_bool();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return parameter.as_int();
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return parameter.as_double();
    case rclcpp::ParameterType::PARAMETER_STRING:
      return parameter.as_string();
    case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
      return parameter.as_bool_array();
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return parameter.as_integer_array();
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return parameter.as_double_array();
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return parameter.as_string_array();
    default:
      throw std::invalid_argument("unsupported ROS parameter type");
  }
}

device_manager::ParameterMap parameter_map(
  const std::vector<rcl_interfaces::msg::Parameter> & messages)
{
  device_manager::ParameterMap result;
  for (const auto & message : messages) {
    result.insert_or_assign(message.name, parameter_value(message));
  }
  return result;
}

rcl_interfaces::msg::Parameter parameter_message(
  const std::string & name, const device_manager::ParameterValue & value)
{
  return std::visit(
    [&name](const auto & typed_value) {
      return rclcpp::Parameter(name, typed_value).to_parameter_msg();
    }, value);
}

builtin_interfaces::msg::Time time_message(device_manager::Clock::time_point time)
{
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
    time.time_since_epoch()).count();
  return static_cast<builtin_interfaces::msg::Time>(
    rclcpp::Time(nanoseconds, RCL_SYSTEM_TIME));
}

device_manager_msgs::msg::DeviceEvent event_message(
  const device_manager::DeviceEvent & event)
{
  device_manager_msgs::msg::DeviceEvent result;
  result.timestamp = time_message(event.timestamp);
  result.level = static_cast<std::uint8_t>(event.level);
  result.code = event.code;
  result.message = event.message;
  result.source = event.source;
  result.target_state = static_cast<std::uint8_t>(event.target_state);
  for (const auto & value : event.values) {
    diagnostic_msgs::msg::KeyValue message;
    message.key = value.first;
    message.value = value.second;
    result.values.push_back(std::move(message));
  }
  return result;
}

device_manager_msgs::msg::TransitionRequest transition_message(
  const device_manager::TransitionRequest & request)
{
  device_manager_msgs::msg::TransitionRequest result;
  result.from_state = static_cast<std::uint8_t>(request.from_state);
  result.target_state = static_cast<std::uint8_t>(request.target_state);
  for (const auto & parameter : request.parameters) {
    result.parameters.push_back(parameter_message(parameter.first, parameter.second));
  }
  return result;
}

device_manager_msgs::msg::DeviceState device_message(
  const device_manager::DeviceObservation & device)
{
  device_manager_msgs::msg::DeviceState result;
  result.id = device.id;
  result.type = device.type;
  result.enabled = device.enabled;
  result.lifecycle_state = static_cast<std::uint8_t>(device.state);
  for (const auto & parameter : device.parameters) {
    result.parameters.push_back(parameter_message(parameter.first, parameter.second));
  }
  if (device.latest_event) {
    result.has_latest_event = true;
    result.latest_event = event_message(*device.latest_event);
  }
  if (device.last_transition) {
    result.has_last_transition = true;
    result.last_transition.request = transition_message(device.last_transition->request);
    result.last_transition.result = static_cast<std::uint8_t>(device.last_transition->result);
    result.last_transition.discarded = device.last_transition->discarded;
    result.last_transition.actual_state =
      static_cast<std::uint8_t>(device.last_transition->actual_state);
    result.last_transition.message = device.last_transition->message;
    result.last_transition.timestamp = time_message(device.last_transition->timestamp);
  }
  return result;
}

}  // namespace

Ros2DeviceManagerApi::Ros2DeviceManagerApi(
  device_manager::IDeviceManagerApplication & application, rclcpp::Node & node,
  std::chrono::milliseconds publish_period, std::string api_prefix)
: DeviceManagerApi(application), node_(node), publish_period_(publish_period),
  api_prefix_(std::move(api_prefix))
{
  if (publish_period_ <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("publish period must be positive");
  }
  if (api_prefix_.empty()) {
    throw std::invalid_argument("api prefix must not be empty");
  }
}

Ros2DeviceManagerApi::~Ros2DeviceManagerApi()
{
  stop();
}

void Ros2DeviceManagerApi::start()
{
  if (publisher_) {
    return;
  }

  publisher_ = node_.create_publisher<device_manager_msgs::msg::DeviceStateArray>(
    api_prefix_ + "/devices", rclcpp::QoS(1).reliable().transient_local());
  get_devices_service_ = node_.create_service<GetDevices>(
    api_prefix_ + "/get_devices",
    [this](const std::shared_ptr<GetDevices::Request>,
    std::shared_ptr<GetDevices::Response> response) {
      response->result = snapshot_message();
    });
  change_state_service_ = node_.create_service<ChangeDeviceState>(
    api_prefix_ + "/change_device_state",
    [this](const std::shared_ptr<ChangeDeviceState::Request> request,
    std::shared_ptr<ChangeDeviceState::Response> response) {
      handle_change_state(request, response);
    });
  patch_parameters_service_ = node_.create_service<PatchDeviceParameters>(
    api_prefix_ + "/patch_parameters",
    [this](const std::shared_ptr<PatchDeviceParameters::Request> request,
    std::shared_ptr<PatchDeviceParameters::Response> response) {
      handle_patch_parameters(request, response);
    });
  publish_timer_ = node_.create_wall_timer(
    publish_period_, [this]() {publish_snapshot();});
}

void Ros2DeviceManagerApi::stop()
{
  publish_timer_.reset();
  patch_parameters_service_.reset();
  change_state_service_.reset();
  get_devices_service_.reset();
  publisher_.reset();
}

device_manager_msgs::msg::DeviceStateArray Ros2DeviceManagerApi::snapshot_message()
{
  device_manager_msgs::msg::DeviceStateArray result;
  result.timestamp = node_.now();
  for (const auto & device : application().devices()) {
    result.devices.push_back(device_message(device));
  }
  return result;
}

void Ros2DeviceManagerApi::publish_snapshot()
{
  publisher_->publish(snapshot_message());
}

void Ros2DeviceManagerApi::handle_change_state(
  const std::shared_ptr<ChangeDeviceState::Request> request,
  std::shared_ptr<ChangeDeviceState::Response> response)
{
  const auto request_priority = priority(request->priority);
  if (!request_priority) {
    response->message = "invalid request priority";
    return;
  }
  std::vector<device_manager::TransitionRequest> requests;
  requests.reserve(request->requests.size());
  try {
    for (const auto & message : request->requests) {
      const auto from = lifecycle_state(message.from_state);
      const auto target = lifecycle_state(message.target_state);
      if (!from || !target) {
        response->message = "invalid lifecycle state";
        return;
      }
      requests.push_back({*from, *target, parameter_map(message.parameters)});
    }
  } catch (const std::exception & exception) {
    response->message = exception.what();
    return;
  }
  const auto result = application().change_state(
    request->device_id, std::move(requests), *request_priority);
  response->accepted = result.accepted;
  response->message = result.message;
}

void Ros2DeviceManagerApi::handle_patch_parameters(
  const std::shared_ptr<PatchDeviceParameters::Request> request,
  std::shared_ptr<PatchDeviceParameters::Response> response)
{
  const auto request_priority = priority(request->priority);
  if (!request_priority) {
    response->message = "invalid request priority";
    return;
  }
  try {
    const auto result = application().patch_parameters(
      request->device_id, parameter_map(request->parameters), *request_priority);
    response->accepted = result.accepted;
    response->message = result.message;
  } catch (const std::exception & exception) {
    response->message = exception.what();
  }
}

}  // namespace device_manager_ros
