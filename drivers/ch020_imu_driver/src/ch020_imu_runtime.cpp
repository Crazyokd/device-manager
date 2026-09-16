#include "ch020_imu_driver/ch020_imu_runtime.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace ch020_imu_driver
{
namespace
{

using device_manager::LifecycleState;
using device_manager::ParameterMap;
using device_manager::TransitionOutcome;
using device_manager::TransitionRequest;
using device_manager::TransitionResult;

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

Ch020ImuRuntime::Ch020ImuRuntime(
  Ch020ImuConfig config,
  Ch020ImuDriver::TransportFactory transport_factory)
: config_(std::move(config)),
  transport_factory_(std::move(transport_factory)),
  state_(LifecycleState::kFinalized)
{
}

void Ch020ImuRuntime::initialize(const device_manager::DeviceDefinition & definition)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (const auto found = definition.parameters.find("imu.frame_id");
    found != definition.parameters.end())
  {
    if (const auto value = std::get_if<std::string>(&found->second)) {
      frame_id_ = *value;
    }
  }
  if (const auto found = definition.parameters.find("imu.use_hardware_time");
    found != definition.parameters.end())
  {
    if (const auto value = std::get_if<bool>(&found->second)) {
      use_hardware_time_ = *value;
    }
  }
  if (packet_handler_) {
    return;
  }

  output_node_ = std::make_shared<rclcpp::Node>(
    "ch020_imu_runtime", rclcpp::NodeOptions().use_global_arguments(false));
  imu_publisher_ = output_node_->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  mag_publisher_ = output_node_->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
  pressure_publisher_ = output_node_->create_publisher<sensor_msgs::msg::FluidPressure>(
    "/imu/pressure", 10);
  packet_handler_ = [this](const ImuSolPacket & packet) {publish_packet(packet);};
}

LifecycleState Ch020ImuRuntime::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

TransitionOutcome Ch020ImuRuntime::materialize(const ParameterMap &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LifecycleState::kFinalized || driver_) {
    return {TransitionResult::kFailure, "runtime is already materialized"};
  }
  driver_ = std::make_unique<Ch020ImuDriver>(config_, transport_factory_);
  driver_->set_packet_handler(packet_handler_);
  driver_->set_connection_handler(
    [this](bool connected, const std::string & message) {
      if (connected) {
        emit_ok_event("ch020_imu.online", message);
      } else {
        emit_error_event("ch020_imu.disconnected", message, LifecycleState::kActive);
      }
    });
  state_ = LifecycleState::kUnconfigured;
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome Ch020ImuRuntime::dematerialize()
{
  std::unique_ptr<Ch020ImuDriver> driver;
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

void Ch020ImuRuntime::set_event_handler(device_manager::EventHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  event_handler_ = std::move(handler);
}

void Ch020ImuRuntime::register_hook(device_manager::HookRegistrar registrar)
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

void Ch020ImuRuntime::set_packet_handler(Ch020ImuDriver::PacketHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  packet_handler_ = std::move(handler);
  if (driver_) {
    driver_->set_packet_handler(packet_handler_);
  }
}

Ch020ImuSnapshot Ch020ImuRuntime::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return driver_ ? driver_->snapshot() : Ch020ImuSnapshot{};
}

TransitionOutcome Ch020ImuRuntime::configure(const ParameterMap & parameters)
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
      "ch020_imu.connect_failed",
      error_message,
      LifecycleState::kUnconfigured);
    return {TransitionResult::kError, error_message};
  }

  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome Ch020ImuRuntime::activate()
{
  Ch020ImuDriver * driver;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kInactive || !driver_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    if (!driver_->is_configured()) {
      return {TransitionResult::kFailure, "imu driver is not configured"};
    }
    state_ = LifecycleState::kActive;
    driver = driver_.get();
  }
  // start() 只启动读线程、不等待回调；在锁外调用，避免读线程事件回调持锁等待。
  driver->start();
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome Ch020ImuRuntime::deactivate()
{
  Ch020ImuDriver * driver;
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

TransitionOutcome Ch020ImuRuntime::cleanup(const ParameterMap &)
{
  Ch020ImuDriver * driver;
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

Ch020ImuConfig Ch020ImuRuntime::config_from(
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

void Ch020ImuRuntime::emit_event(device_manager::DeviceEvent event)
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

void Ch020ImuRuntime::emit_error_event(
  std::string code,
  std::string message,
  LifecycleState target_state)
{
  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = device_manager::EventLevel::kError;
  event.code = std::move(code);
  event.message = std::move(message);
  event.source = "ch020_imu_driver";
  event.target_state = target_state;
  emit_event(std::move(event));
}

void Ch020ImuRuntime::emit_ok_event(std::string code, std::string message)
{
  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = device_manager::EventLevel::kOk;
  event.code = std::move(code);
  event.message = std::move(message);
  event.source = "ch020_imu_driver";
  event.target_state = LifecycleState::kUnknown;
  emit_event(std::move(event));
}

rclcpp::Time Ch020ImuRuntime::make_stamp(
  std::int64_t local_ns, std::uint32_t hardware_timestamp_ms)
{
  if (!use_hardware_time_) {
    return rclcpp::Time(local_ns, RCL_ROS_TIME);
  }
  const auto sensor_ns = static_cast<std::int64_t>(hardware_timestamp_ms) * 1000000LL;
  const auto offset = local_ns - sensor_ns;
  time_sync_.push_back(offset);
  const auto synchronized_ns = time_sync_.get_min() + sensor_ns;
  if (synchronized_ns > local_ns + 100000000LL || synchronized_ns < local_ns - 100000000LL) {
    time_sync_.reset();
    return rclcpp::Time(local_ns, RCL_ROS_TIME);
  }
  return rclcpp::Time(synchronized_ns, RCL_ROS_TIME);
}

void Ch020ImuRuntime::publish_packet(const ImuSolPacket & packet)
{
  const auto now = output_node_->now();
  const auto stamp = make_stamp(now.nanoseconds(), packet.timestamp_ms);

  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = frame_id_;
  imu.orientation.w = packet.quat_w;
  imu.orientation.x = packet.quat_x;
  imu.orientation.y = packet.quat_y;
  imu.orientation.z = packet.quat_z;
  imu.orientation_covariance.fill(0.0);
  imu.angular_velocity.x = packet.gyro_x;
  imu.angular_velocity.y = packet.gyro_y;
  imu.angular_velocity.z = packet.gyro_z;
  imu.angular_velocity_covariance.fill(0.0);
  imu.linear_acceleration.x = packet.acc_x;
  imu.linear_acceleration.y = packet.acc_y;
  imu.linear_acceleration.z = packet.acc_z;
  imu.linear_acceleration_covariance.fill(0.0);
  imu_publisher_->publish(imu);

  if (packet.mag_x != 0.0F || packet.mag_y != 0.0F || packet.mag_z != 0.0F) {
    sensor_msgs::msg::MagneticField magnetic_field;
    magnetic_field.header.stamp = stamp;
    magnetic_field.header.frame_id = frame_id_;
    magnetic_field.magnetic_field.x = static_cast<double>(packet.mag_x) * 1e-6;
    magnetic_field.magnetic_field.y = static_cast<double>(packet.mag_y) * 1e-6;
    magnetic_field.magnetic_field.z = static_cast<double>(packet.mag_z) * 1e-6;
    mag_publisher_->publish(magnetic_field);
  }

  if (packet.pressure != 0.0F) {
    sensor_msgs::msg::FluidPressure pressure;
    pressure.header.stamp = stamp;
    pressure.header.frame_id = frame_id_;
    pressure.fluid_pressure = static_cast<double>(packet.pressure);
    pressure_publisher_->publish(pressure);
  }
}

}  // namespace ch020_imu_driver

PLUGINLIB_EXPORT_CLASS(
  ch020_imu_driver::Ch020ImuRuntime,
  device_manager::IDeviceRuntime)
