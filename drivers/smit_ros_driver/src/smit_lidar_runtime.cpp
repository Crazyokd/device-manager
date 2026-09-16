#include "smit_ros_driver/smit_lidar_runtime.hpp"

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

namespace smit_ros_driver
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
constexpr double kFullCircleDegrees = 360.0;
constexpr double kFullCircleToleranceDegrees = 1e-3;

double normalize_raw_angle(double angle_deg)
{
  double normalized = std::fmod(angle_deg, kFullCircleDegrees);
  if (normalized < 0.0) {
    normalized += kFullCircleDegrees;
  }
  return normalized;
}

double raw_angle_to_signed(double raw_angle_deg)
{
  const double normalized = normalize_raw_angle(raw_angle_deg);
  return normalized >= 180.0 ? normalized - kFullCircleDegrees : normalized;
}

bool is_full_circle_window(double angle_min_deg, double angle_max_deg)
{
  return angle_max_deg - angle_min_deg >=
         kFullCircleDegrees - kFullCircleToleranceDegrees;
}

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

std::optional<std::int64_t> integer_parameter(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  return parameter_as<std::int64_t>(parameters, name, error_message);
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

std::optional<std::string> string_parameter(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  return parameter_as<std::string>(parameters, name, error_message);
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

SmitLidarRuntime::SmitLidarRuntime(
  SmitLidarConfig config,
  SmitLidarDriver::TransportFactory transport_factory)
: config_(std::move(config)),
  transport_factory_(std::move(transport_factory)),
  state_(LifecycleState::kFinalized)
{
}

void SmitLidarRuntime::initialize(const device_manager::DeviceDefinition & definition)
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
  if (scan_handler_ || !rclcpp::ok()) {
    return;
  }

  output_node_ = std::make_shared<rclcpp::Node>(
    "smit_lidar_runtime", rclcpp::NodeOptions().use_global_arguments(false));
  scan_publisher_ = output_node_->create_publisher<sensor_msgs::msg::LaserScan>(
    topic_, rclcpp::SensorDataQoS());
  scan_handler_ = [this](const SmitScan & scan) {publish_scan(scan);};
}

LifecycleState SmitLidarRuntime::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

TransitionOutcome SmitLidarRuntime::materialize(const ParameterMap &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LifecycleState::kFinalized || driver_) {
    return {TransitionResult::kFailure, "runtime is already materialized"};
  }
  driver_ = std::make_unique<SmitLidarDriver>(config_, transport_factory_);
  driver_->set_scan_handler(scan_handler_);
  driver_->set_config_handler(config_handler_);
  driver_->set_connection_handler(
    [this](bool connected, const std::string & message) {
      if (connected) {
        emit_ok_event("smit_lidar.online", message);
      } else {
        emit_error_event("smit_lidar.disconnected", message, LifecycleState::kActive);
      }
    });
  state_ = LifecycleState::kUnconfigured;
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome SmitLidarRuntime::dematerialize()
{
  std::unique_ptr<SmitLidarDriver> driver;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!driver_ || state_ == LifecycleState::kFinalized) {
      return {TransitionResult::kFailure, "runtime is not materialized"};
    }
    state_ = LifecycleState::kShuttingDown;
    driver = std::move(driver_);
  }
  driver->cleanup();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = LifecycleState::kFinalized;
  }
  return {TransitionResult::kSuccess, {}};
}

void SmitLidarRuntime::set_event_handler(device_manager::EventHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  event_handler_ = std::move(handler);
}

void SmitLidarRuntime::register_hook(device_manager::HookRegistrar registrar)
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

void SmitLidarRuntime::set_scan_handler(SmitLidarDriver::ScanHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  scan_handler_ = std::move(handler);
  if (driver_) {
    driver_->set_scan_handler(scan_handler_);
  }
}

void SmitLidarRuntime::set_config_handler(SmitLidarDriver::ConfigHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_handler_ = std::move(handler);
  if (driver_) {
    driver_->set_config_handler(config_handler_);
  }
}

SmitLidarSnapshot SmitLidarRuntime::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return driver_ ? driver_->snapshot() : SmitLidarSnapshot{};
}

TransitionOutcome SmitLidarRuntime::configure(const ParameterMap & parameters)
{
  std::string error_message;
  bool configured;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kUnconfigured || !driver_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    const auto config = config_from(parameters, error_message);
    if (!error_message.empty()) {
      return {TransitionResult::kFailure, error_message};
    }
    state_ = LifecycleState::kConfiguring;
    configured = driver_->configure(config, error_message);
    if (configured) {
      state_ = LifecycleState::kInactive;
    } else {
      state_ = LifecycleState::kUnconfigured;
    }
  }

  if (!configured) {
    emit_error_event(
      "smit_lidar.connect_failed",
      error_message,
      LifecycleState::kUnconfigured);
    return {TransitionResult::kError, error_message};
  }

  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome SmitLidarRuntime::activate()
{
  SmitLidarDriver * driver;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kInactive || !driver_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    if (!driver_->is_configured()) {
      return {TransitionResult::kFailure, "lidar driver is not configured"};
    }
    state_ = LifecycleState::kActive;
    driver = driver_.get();
  }
  // start() 只启动读线程、不等待回调；在锁外调用，避免读线程事件回调持锁等待。
  driver->start();
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome SmitLidarRuntime::deactivate()
{
  SmitLidarDriver * driver;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kActive || !driver_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    state_ = LifecycleState::kInactive;
    driver = driver_.get();
  }
  // stop() 会 join 读线程；读线程可能正回调事件（需要拿 mutex_），必须在锁外调用。
  driver->stop();
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome SmitLidarRuntime::cleanup(const ParameterMap &)
{
  SmitLidarDriver * driver;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kInactive || !driver_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    state_ = LifecycleState::kCleaningUp;
    driver = driver_.get();
  }
  driver->cleanup();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = LifecycleState::kUnconfigured;
  }
  return {TransitionResult::kSuccess, {}};
}

SmitLidarConfig SmitLidarRuntime::config_from(
  const ParameterMap & parameters,
  std::string & error_message) const
{
  auto config = driver_->config();
  if (const auto value = string_parameter(
      parameters, "device.interface.serial_port", error_message))
  {
    config.port = *value;
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(
      parameters, "device.interface.serial_baudrate", error_message))
  {
    config.baudrate = static_cast<int>(*value);
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
  if (const auto value = integer_parameter(parameters, "read_chunk_size", error_message)) {
    config.read_chunk_size = static_cast<std::size_t>(*value);
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(parameters, "data_timeout_ms", error_message)) {
    config.data_timeout_ms = static_cast<int>(*value);
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(parameters, "reconnect_backoff_ms", error_message)) {
    config.reconnect_backoff_ms = static_cast<int>(*value);
  }
  return config;
}

void SmitLidarRuntime::emit_event(device_manager::DeviceEvent event)
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

void SmitLidarRuntime::emit_error_event(
  std::string code,
  std::string message,
  LifecycleState target_state)
{
  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = device_manager::EventLevel::kError;
  event.code = std::move(code);
  event.message = std::move(message);
  event.source = "smit_lidar_driver";
  event.target_state = target_state;
  emit_event(std::move(event));
}

void SmitLidarRuntime::emit_ok_event(std::string code, std::string message)
{
  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = device_manager::EventLevel::kOk;
  event.code = std::move(code);
  event.message = std::move(message);
  event.source = "smit_lidar_driver";
  event.target_state = LifecycleState::kUnknown;
  emit_event(std::move(event));
}

void SmitLidarRuntime::publish_scan(const SmitScan & scan)
{
  if (scan.motor_speed <= 0 || scan.lidar_freq <= 0) {
    return;
  }

  const double angle_increment =
    2.0 * kPi * static_cast<double>(scan.motor_speed) /
    static_cast<double>(scan.lidar_freq);
  if (angle_increment <= 0.0) {
    return;
  }

  const double min_deg = scan.angle_min_deg;
  const double max_deg = scan.angle_max_deg;
  const bool full_circle = is_full_circle_window(min_deg, max_deg);
  const double start_raw_deg = normalize_raw_angle(min_deg);
  const double window_width_deg = full_circle ? kFullCircleDegrees : max_deg - min_deg;
  const std::size_t sample_count = static_cast<std::size_t>(
    std::floor((window_width_deg * kDegreesToRadians) / angle_increment + 0.5)) + 1U;
  const double time_increment = 1.0 / static_cast<double>(scan.lidar_freq);
  const auto scan_duration_ns = static_cast<std::int64_t>(
    std::llround(std::abs(time_increment) * static_cast<double>(sample_count) * 1e9));

  sensor_msgs::msg::LaserScan message;
  // Keep the timestamp aligned with the midpoint of the scan.
  message.header.stamp =
    output_node_->now() - rclcpp::Duration::from_nanoseconds(scan_duration_ns / 2);
  message.header.frame_id = frame_id_;
  message.angle_min = static_cast<float>(min_deg * kDegreesToRadians);
  message.angle_max = static_cast<float>(max_deg * kDegreesToRadians);
  message.angle_increment = static_cast<float>(angle_increment);
  message.time_increment = static_cast<float>(time_increment);
  message.scan_time = static_cast<float>(1.0 / static_cast<double>(scan.motor_speed));
  message.range_min = 0.05F;
  message.range_max = 50.0F;
  message.ranges.assign(sample_count, std::numeric_limits<float>::infinity());
  message.intensities.assign(sample_count, 0.0F);

  for (const SmitPoint & point : scan.points) {
    double offset_deg = normalize_raw_angle(point.angle_deg) - start_raw_deg;
    if (offset_deg < 0.0) {
      offset_deg += kFullCircleDegrees;
    }
    if (!full_circle) {
      const double scan_angle_deg = raw_angle_to_signed(point.angle_deg);
      if (scan_angle_deg < min_deg || scan_angle_deg > max_deg) {
        continue;
      }
      offset_deg = scan_angle_deg - min_deg;
      if (offset_deg < 0.0) {
        offset_deg += kFullCircleDegrees;
      }
    }
    const auto sample_index = static_cast<std::size_t>(
      std::llround((offset_deg * kDegreesToRadians) / angle_increment));
    if (sample_index >= sample_count) {
      continue;
    }
    if (!std::isfinite(message.ranges[sample_index]) ||
      point.distance_m < message.ranges[sample_index])
    {
      message.ranges[sample_index] = point.distance_m;
      message.intensities[sample_index] = point.intensity;
    }
  }

  scan_publisher_->publish(message);
}

}  // namespace smit_ros_driver

PLUGINLIB_EXPORT_CLASS(
  smit_ros_driver::SmitLidarRuntime,
  device_manager::IDeviceRuntime)
