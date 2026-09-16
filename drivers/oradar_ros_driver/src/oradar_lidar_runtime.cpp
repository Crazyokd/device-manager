#include "oradar_ros_driver/oradar_lidar_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace oradar_ros_driver
{
namespace
{

using device_manager::LifecycleState;
using device_manager::ParameterMap;
using device_manager::TransitionOutcome;
using device_manager::TransitionRequest;
using device_manager::TransitionResult;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kPhysicalFovDegrees = 270.0;

template<typename T>
std::optional<T> parameter_as(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    return std::nullopt;
  }
  if (const auto value = std::get_if<T>(&found->second)) {
    return *value;
  }
  error_message = "invalid parameter type: " + name;
  return std::nullopt;
}

std::optional<std::string> string_parameter(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  return parameter_as<std::string>(parameters, name, error_message);
}

std::optional<std::string> string_parameter(
  const ParameterMap & parameters,
  const std::vector<std::string> & names,
  std::string & error_message)
{
  for (const auto & name : names) {
    const auto value = string_parameter(parameters, name, error_message);
    if (value || !error_message.empty()) {
      return value;
    }
  }
  return std::nullopt;
}

std::optional<std::int64_t> integer_parameter(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  return parameter_as<std::int64_t>(parameters, name, error_message);
}

std::optional<std::int64_t> integer_parameter(
  const ParameterMap & parameters,
  const std::vector<std::string> & names,
  std::string & error_message)
{
  for (const auto & name : names) {
    const auto value = integer_parameter(parameters, name, error_message);
    if (value || !error_message.empty()) {
      return value;
    }
  }
  return std::nullopt;
}

std::optional<double> number_parameter(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    return std::nullopt;
  }
  if (const auto value = std::get_if<double>(&found->second)) {
    return *value;
  }
  if (const auto value = std::get_if<std::int64_t>(&found->second)) {
    return static_cast<double>(*value);
  }
  error_message = "invalid parameter type: " + name;
  return std::nullopt;
}

const char * error_name(ord_sdk::error_t error)
{
  switch (error) {
    case ord_sdk::no_error:
      return "no_error";
    case ord_sdk::operation_failure:
      return "operation_failure";
    case ord_sdk::timed_out:
      return "timed_out";
    case ord_sdk::address_in_use:
      return "address_in_use";
  }
  return "unknown";
}

bool event_needs_recovery(const device_manager::DeviceEvent & event)
{
  return event.level == device_manager::EventLevel::kError ||
         event.level == device_manager::EventLevel::kStale;
}

std::vector<TransitionRequest> recovery_requests(LifecycleState from, LifecycleState target)
{
  if (target == LifecycleState::kFinalized) {
    if (from == LifecycleState::kActive) {
      return {
        {LifecycleState::kActive, LifecycleState::kInactive, {}},
        {LifecycleState::kInactive, LifecycleState::kUnconfigured, {}},
        {LifecycleState::kUnconfigured, LifecycleState::kFinalized, {}},
      };
    }
    if (from == LifecycleState::kInactive) {
      return {
        {LifecycleState::kInactive, LifecycleState::kUnconfigured, {}},
        {LifecycleState::kUnconfigured, LifecycleState::kFinalized, {}},
      };
    }
    if (from == LifecycleState::kUnconfigured) {
      return {{LifecycleState::kUnconfigured, LifecycleState::kFinalized, {}}};
    }
  }
  if (from == LifecycleState::kActive && target == LifecycleState::kInactive) {
    return {{LifecycleState::kActive, LifecycleState::kInactive, {}}};
  }
  if (from == LifecycleState::kActive && target == LifecycleState::kUnconfigured) {
    return {
      {LifecycleState::kActive, LifecycleState::kInactive, {}},
      {LifecycleState::kInactive, LifecycleState::kUnconfigured, {}},
    };
  }
  if (from == LifecycleState::kInactive && target == LifecycleState::kUnconfigured) {
    return {{LifecycleState::kInactive, LifecycleState::kUnconfigured, {}}};
  }
  return {};
}

}  // namespace

OradarLidarRuntime::OradarLidarRuntime(OradarLidarClientFactory client_factory)
: client_factory_(std::move(client_factory)),
  state_(LifecycleState::kFinalized)
{
  if (!client_factory_) {
    client_factory_ = [](const OradarLidarConfig & config) {
        return make_ord_driver_client(config);
      };
  }
}

OradarLidarRuntime::~OradarLidarRuntime()
{
  stop_reader();
}

void OradarLidarRuntime::initialize(const device_manager::DeviceDefinition & definition)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool is_front = definition.id.find("front") != std::string::npos;
  topic_ = is_front ? "scan" : "scan_rear";
  frame_id_ = is_front ? "right_front_laser_link" : "left_behind_laser_link";
  if (const auto found = definition.parameters.find("lidar.topic");
    found != definition.parameters.end())
  {
    if (const auto value = std::get_if<std::string>(&found->second)) {
      topic_ = *value;
    }
  }
  if (const auto found = definition.parameters.find("lidar.frame_id");
    found != definition.parameters.end())
  {
    if (const auto value = std::get_if<std::string>(&found->second)) {
      frame_id_ = *value;
    }
  }
  if (scan_publisher_ || !rclcpp::ok()) {
    return;
  }

  output_node_ = std::make_shared<rclcpp::Node>(
    "oradar_lidar_runtime", rclcpp::NodeOptions().use_global_arguments(false));
  scan_publisher_ = output_node_->create_publisher<sensor_msgs::msg::LaserScan>(
    topic_, rclcpp::SensorDataQoS());
}

LifecycleState OradarLidarRuntime::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

TransitionOutcome OradarLidarRuntime::materialize(const ParameterMap &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LifecycleState::kFinalized) {
    return {TransitionResult::kFailure, "runtime is already materialized"};
  }
  state_ = LifecycleState::kUnconfigured;
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome OradarLidarRuntime::dematerialize()
{
  stop_reader();
  std::unique_ptr<OradarLidarClient> client;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == LifecycleState::kFinalized) {
      return {TransitionResult::kFailure, "runtime is not materialized"};
    }
    state_ = LifecycleState::kShuttingDown;
    client = std::move(client_);
  }
  if (client) {
    client->close();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = LifecycleState::kFinalized;
  }
  return {TransitionResult::kSuccess, {}};
}

void OradarLidarRuntime::set_event_handler(device_manager::EventHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  event_handler_ = std::move(handler);
}

void OradarLidarRuntime::register_hook(device_manager::HookRegistrar registrar)
{
  auto handled_event_revision = std::make_shared<std::uint64_t>(0U);
  registrar(
    [handled_event_revision](const device_manager::HookContext & context) {
      if (!context.enabled) {
        return recovery_requests(context.state, LifecycleState::kFinalized);
      }
      if (context.state == LifecycleState::kFinalized) {
        return std::vector<TransitionRequest>{
          {LifecycleState::kFinalized, LifecycleState::kUnconfigured, context.parameters}};
      }
      if (context.state == LifecycleState::kUnconfigured) {
        return std::vector<TransitionRequest>{
          {LifecycleState::kUnconfigured, LifecycleState::kInactive, context.parameters}};
      }
      if (context.state == LifecycleState::kInactive) {
        return std::vector<TransitionRequest>{
          {LifecycleState::kInactive, LifecycleState::kActive, {}}};
      }
      if (context.state != LifecycleState::kActive || !context.latest_event ||
        !event_needs_recovery(*context.latest_event) ||
        context.latest_event_revision <= *handled_event_revision)
      {
        return std::vector<TransitionRequest>{};
      }

      *handled_event_revision = context.latest_event_revision;
      return recovery_requests(context.state, context.latest_event->target_state);
    });
}

void OradarLidarRuntime::set_scan_handler(ScanHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  scan_handler_ = std::move(handler);
}

OradarLidarConfig OradarLidarRuntime::config() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

TransitionOutcome OradarLidarRuntime::configure(const ParameterMap & parameters)
{
  std::string error_message;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kUnconfigured) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    config_ = config_from(parameters, error_message);
    if (!error_message.empty()) {
      return {TransitionResult::kFailure, error_message};
    }
    state_ = LifecycleState::kConfiguring;
    try {
      client_ = client_factory_(config_);
      client_->set_timeout(config_.timeout_ms);
    } catch (const std::exception & error) {
      state_ = LifecycleState::kUnconfigured;
      return {TransitionResult::kFailure, error.what()};
    }
  }

  if (!connect_client(error_message) || !apply_device_settings(error_message)) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = LifecycleState::kUnconfigured;
    }
    emit_error_event("oradar_lidar.connect_failed", error_message, LifecycleState::kUnconfigured);
    return {TransitionResult::kError, error_message};
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = LifecycleState::kInactive;
  }
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome OradarLidarRuntime::activate()
{
  OradarLidarClient * client;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kInactive || !client_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    client = client_.get();
  }

  if (client->enable_measure() != ord_sdk::no_error ||
    client->enable_data_stream() != ord_sdk::no_error)
  {
    return {TransitionResult::kError, "failed to enable MS500 measurement stream"};
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = LifecycleState::kActive;
    read_thread_running_ = true;
    connected_event_emitted_ = false;
    read_thread_ = std::thread([this]() {read_loop();});
  }
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome OradarLidarRuntime::deactivate()
{
  OradarLidarClient * client = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kActive || !client_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    state_ = LifecycleState::kInactive;
    client = client_.get();
  }
  stop_reader();
  client->disable_data_stream();
  client->disable_measure();
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome OradarLidarRuntime::cleanup(const ParameterMap &)
{
  std::unique_ptr<OradarLidarClient> client;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kInactive) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    state_ = LifecycleState::kCleaningUp;
    client = std::move(client_);
  }
  if (client) {
    client->close();
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = LifecycleState::kUnconfigured;
  }
  return {TransitionResult::kSuccess, {}};
}

OradarLidarConfig OradarLidarRuntime::config_from(
  const ParameterMap & parameters,
  std::string & error_message) const
{
  OradarLidarConfig config;
  if (const auto value = string_parameter(
      parameters,
      std::vector<std::string>{
        "device.interface.ip_address",
        "device.interface.ethernet_ip",
      },
      error_message))
  {
    config.ip_address = *value;
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = string_parameter(
      parameters,
      std::vector<std::string>{
        "device.interface.name",
        "device.interface.network_interface",
        "device.interface.ethernet_name",
      },
      error_message))
  {
    config.network_interface = *value;
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(
      parameters,
      std::vector<std::string>{
        "device.interface.udp_port",
        "device.interface.ethernet_port",
      },
      error_message))
  {
    config.udp_port = static_cast<int>(*value);
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(parameters, "lidar.scan_frequency", error_message)) {
    config.motor_speed = static_cast<int>(*value);
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(parameters, "lidar.filter_size", error_message)) {
    config.filter_size = static_cast<int>(*value);
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(parameters, "lidar.motor_dir", error_message)) {
    config.motor_dir = static_cast<int>(*value);
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = number_parameter(parameters, "lidar.scan_angle_min", error_message)) {
    config.angle_min_deg = *value;
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = number_parameter(parameters, "lidar.scan_angle_max", error_message)) {
    config.angle_max_deg = *value;
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = number_parameter(parameters, "lidar.range_min", error_message)) {
    config.range_min_m = *value;
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = number_parameter(parameters, "lidar.range_max", error_message)) {
    config.range_max_m = *value;
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = parameter_as<bool>(parameters, "lidar.inverted", error_message)) {
    config.inverted = *value;
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(parameters, "data_timeout_ms", error_message)) {
    config.timeout_ms = static_cast<int>(*value);
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(parameters, "reconnect_backoff_ms", error_message)) {
    config.reconnect_backoff_ms = static_cast<int>(*value);
  }
  return config;
}

bool OradarLidarRuntime::connect_client(std::string & error_message)
{
  OradarLidarClient * client;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    client = client_.get();
  }
  if (!client) {
    error_message = "MS500 client is not configured";
    return false;
  }
  if (!client->is_open()) {
    const auto open_result = client->open();
    if (open_result != ord_sdk::no_error) {
      error_message = std::string("open failed: ") + error_name(open_result);
      return false;
    }
  }
  const auto connect_result = client->track_connect();
  if (connect_result != ord_sdk::no_error) {
    error_message = std::string("track connect failed: ") + error_name(connect_result);
    return false;
  }
  return true;
}

bool OradarLidarRuntime::apply_device_settings(std::string & error_message)
{
  OradarLidarClient * client;
  OradarLidarConfig config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    client = client_.get();
    config = config_;
  }
  if (!client) {
    error_message = "MS500 client is not configured";
    return false;
  }

  bool changed = false;
  std::uint32_t value = 0;
  if (client->get_scan_speed(value) != ord_sdk::no_error) {
    error_message = "failed to read scan speed";
    return false;
  }
  if (value != static_cast<std::uint32_t>(config.motor_speed)) {
    const auto result = client->set_scan_speed(static_cast<std::uint32_t>(config.motor_speed));
    if (result != ord_sdk::no_error) {
      error_message = std::string("failed to set scan speed: ") + error_name(result);
      return false;
    }
    changed = true;
  }
  if (client->get_tail_filter_level(value) != ord_sdk::no_error) {
    error_message = "failed to read filter size";
    return false;
  }
  if (value != static_cast<std::uint32_t>(config.filter_size)) {
    const auto result = client->set_tail_filter_level(static_cast<std::uint32_t>(config.filter_size));
    if (result != ord_sdk::no_error) {
      error_message = std::string("failed to set filter size: ") + error_name(result);
      return false;
    }
    changed = true;
  }
  if (client->get_scan_direction(value) != ord_sdk::no_error) {
    error_message = "failed to read scan direction";
    return false;
  }
  if (value != static_cast<std::uint32_t>(config.motor_dir)) {
    const auto result = client->set_scan_direction(static_cast<std::uint32_t>(config.motor_dir));
    if (result != ord_sdk::no_error) {
      error_message = std::string("failed to set scan direction: ") + error_name(result);
      return false;
    }
    changed = true;
  }
  if (changed) {
    const auto result = client->apply_configs();
    if (result != ord_sdk::no_error) {
      error_message = std::string("failed to save lidar config: ") + error_name(result);
      return false;
    }
  }
  return true;
}

void OradarLidarRuntime::read_loop()
{
  while (true) {
    OradarLidarClient * client;
    OradarLidarConfig config;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!read_thread_running_ || !client_) {
        break;
      }
      client = client_.get();
      config = config_;
    }

    if (!client->is_open()) {
      std::string error_message;
      if (!connect_client(error_message) || !apply_device_settings(error_message) ||
        client->enable_measure() != ord_sdk::no_error ||
        client->enable_data_stream() != ord_sdk::no_error)
      {
        emit_error_event(
          "oradar_lidar.disconnected",
          error_message.empty() ? "MS500 reconnect failed" : error_message,
          LifecycleState::kActive);
        std::this_thread::sleep_for(std::chrono::milliseconds(config.reconnect_backoff_ms));
        continue;
      }
    }

    ord_sdk::ScanFrameData frame;
    const auto start = output_node_ ? output_node_->now() : rclcpp::Clock(RCL_SYSTEM_TIME).now();
    const auto result = client->get_scan_frame_data(frame);
    const auto end = output_node_ ? output_node_->now() : rclcpp::Clock(RCL_SYSTEM_TIME).now();
    if (result != ord_sdk::no_error) {
      client->close();
      emit_error_event(
        "oradar_lidar.disconnected",
        std::string("read scan frame failed: ") + error_name(result),
        LifecycleState::kActive);
      std::this_thread::sleep_for(std::chrono::milliseconds(config.reconnect_backoff_ms));
      continue;
    }

    bool emit_online = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      emit_online = !connected_event_emitted_;
      connected_event_emitted_ = true;
    }
    if (emit_online) {
      emit_ok_event("oradar_lidar.online", "MS500 scan stream is online");
    }
    publish_scan(to_scan_message(frame, start, (end - start).seconds()));
  }
}

void OradarLidarRuntime::stop_reader()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    read_thread_running_ = false;
  }
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

sensor_msgs::msg::LaserScan OradarLidarRuntime::to_scan_message(
  const ord_sdk::ScanFrameData & frame,
  const rclcpp::Time & stamp,
  double scan_time_seconds) const
{
  OradarLidarConfig config;
  std::string frame_id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config = config_;
    frame_id = frame_id_;
  }

  sensor_msgs::msg::LaserScan message;
  message.header.stamp = stamp;
  message.header.frame_id = frame_id;
  message.angle_min = static_cast<float>(config.angle_min_deg * kDegreesToRadians);
  message.angle_max = static_cast<float>(config.angle_max_deg * kDegreesToRadians);
  message.scan_time = static_cast<float>(std::max(0.0, scan_time_seconds));
  message.range_min = static_cast<float>(config.range_min_m);
  message.range_max = static_cast<float>(config.range_max_m);

  if (frame.layers.empty() || frame.layers[0].ranges.empty()) {
    return message;
  }

  const auto & ranges = frame.layers[0].ranges;
  const auto & intensities = frame.layers[0].intensities;
  const int node_count = static_cast<int>(ranges.size());
  const double angle_range = config.angle_max_deg - config.angle_min_deg;
  int count = static_cast<int>(
    std::llround(static_cast<double>(node_count) * angle_range / kPhysicalFovDegrees));
  count = std::max(0, std::min(count, node_count));
  const int node_start = static_cast<int>(
    std::llround(static_cast<double>(node_count) * (135.0 + config.angle_min_deg) /
    kPhysicalFovDegrees));
  if (count <= 0) {
    return message;
  }

  message.ranges.assign(static_cast<std::size_t>(count), 0.0F);
  message.intensities.assign(static_cast<std::size_t>(count), 0.0F);
  if (count > 1) {
    message.angle_increment = (message.angle_max - message.angle_min) /
      static_cast<float>(count - 1);
  }
  message.time_increment = node_count > 0 ?
    static_cast<float>(std::max(0.0, scan_time_seconds) / static_cast<double>(node_count)) :
    0.0F;

  for (int i = 0; i < count; ++i) {
    const int current_index = node_start + i;
    float range = 0.0F;
    float intensity = 0.0F;
    if (current_index >= 0 && current_index < node_count) {
      range = static_cast<float>(ranges[static_cast<std::size_t>(current_index)]) * 0.002F;
      if (static_cast<std::size_t>(current_index) < intensities.size()) {
        intensity = static_cast<float>(intensities[static_cast<std::size_t>(current_index)]);
      }
      if (range > message.range_max || range < message.range_min) {
        range = 0.0F;
        intensity = 0.0F;
      }
    }
    const auto target_index = config.inverted ? static_cast<std::size_t>(count - 1 - i) :
      static_cast<std::size_t>(i);
    message.ranges[target_index] = range;
    message.intensities[target_index] = intensity;
  }
  return message;
}

void OradarLidarRuntime::publish_scan(const sensor_msgs::msg::LaserScan & message)
{
  ScanHandler handler;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handler = scan_handler_;
    publisher = scan_publisher_;
  }
  if (publisher) {
    publisher->publish(message);
  }
  if (handler) {
    handler(message);
  }
}

void OradarLidarRuntime::emit_event(device_manager::DeviceEvent event)
{
  device_manager::EventHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handler = event_handler_;
  }
  if (handler) {
    handler(std::move(event));
  }
}

void OradarLidarRuntime::emit_error_event(
  std::string code,
  std::string message,
  LifecycleState target_state)
{
  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = device_manager::EventLevel::kError;
  event.code = std::move(code);
  event.message = std::move(message);
  event.source = "oradar_lidar_driver";
  event.target_state = target_state;
  emit_event(std::move(event));
}

void OradarLidarRuntime::emit_ok_event(std::string code, std::string message)
{
  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = device_manager::EventLevel::kOk;
  event.code = std::move(code);
  event.message = std::move(message);
  event.source = "oradar_lidar_driver";
  event.target_state = LifecycleState::kUnknown;
  emit_event(std::move(event));
}

}  // namespace oradar_ros_driver

PLUGINLIB_EXPORT_CLASS(
  oradar_ros_driver::OradarLidarRuntime,
  device_manager::IDeviceRuntime)
