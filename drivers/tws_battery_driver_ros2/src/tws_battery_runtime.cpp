#include "tws_battery_driver_ros2/tws_battery_runtime.hpp"

#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "tws_battery_driver_ros2/battery_state_projection.hpp"

namespace tws_battery_driver_ros2
{
namespace
{

using device_manager::LifecycleState;
using device_manager::ParameterMap;
using device_manager::TransitionOutcome;
using device_manager::TransitionRequest;
using device_manager::TransitionResult;

constexpr float kMilliToUnit = 1000.0F;

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

std::optional<bool> bool_parameter(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  return parameter_as<bool>(parameters, name, error_message);
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

TwsBatteryRuntime::TwsBatteryRuntime(
  BatteryDriverConfig config,
  TwsBatteryDriver::TransportFactory transport_factory)
: config_(std::move(config)),
  transport_factory_(std::move(transport_factory)),
  state_(LifecycleState::kFinalized)
{
}

TwsBatteryRuntime::~TwsBatteryRuntime()
{
  stop_polling();
}

void TwsBatteryRuntime::initialize(const device_manager::DeviceDefinition &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!rclcpp::ok()) {
    return;
  }
  output_node_ = std::make_shared<rclcpp::Node>(
    "tws_battery_runtime", rclcpp::NodeOptions().use_global_arguments(false));
  battery_state_publisher_ = output_node_->create_publisher<sensor_msgs::msg::BatteryState>(
    "/battery/state", rclcpp::SensorDataQoS());
  bms_status_publisher_ = output_node_->create_publisher<tws_battery_driver_ros2::msg::BmsStatus>(
    "/battery/status", 10);
}

LifecycleState TwsBatteryRuntime::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

TransitionOutcome TwsBatteryRuntime::materialize(const ParameterMap &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LifecycleState::kFinalized || driver_) {
    return {TransitionResult::kFailure, "runtime is already materialized"};
  }
  driver_ = std::make_unique<TwsBatteryDriver>(config_, transport_factory_);
  state_ = LifecycleState::kUnconfigured;
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome TwsBatteryRuntime::dematerialize()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!driver_ || state_ == LifecycleState::kFinalized) {
      return {TransitionResult::kFailure, "runtime is not materialized"};
    }
    state_ = LifecycleState::kShuttingDown;
  }
  stop_polling();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    driver_->cleanup();
    driver_.reset();
    state_ = LifecycleState::kFinalized;
  }
  return {TransitionResult::kSuccess, {}};
}

void TwsBatteryRuntime::set_event_handler(device_manager::EventHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  event_handler_ = std::move(handler);
}

void TwsBatteryRuntime::register_hook(device_manager::HookRegistrar registrar)
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

bool TwsBatteryRuntime::poll_once()
{
  BatteryPollResult result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kActive) {
      return false;
    }
    result = driver_->poll_once();
    last_poll_error_ = result.ok ? std::string{} : result.error_message;
  }

  for (const auto & record : result.log_records) {
    LogHandler handler;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      handler = log_handler_;
    }
    if (handler) {
      handler(record);
    }
  }

  if (!result.ok) {
    if (result.report_failure_event) {
      emit_error_event(
        "tws_battery.poll_failed",
        result.error_message.empty() ? result.snapshot.last_error : result.error_message,
        LifecycleState::kUnconfigured);
    }
    return false;
  }
  if (!result.log_records.empty()) {
    emit_ok_event("tws_battery.online", "battery online");
  }
  return true;
}

BatterySnapshot TwsBatteryRuntime::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return driver_ ? driver_->snapshot() : BatterySnapshot{};
}

int TwsBatteryRuntime::poll_interval_ms() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return driver_ ? driver_->config().poll_interval_ms : config_.poll_interval_ms;
}

void TwsBatteryRuntime::set_log_handler(LogHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  log_handler_ = std::move(handler);
}

TransitionOutcome TwsBatteryRuntime::configure(const ParameterMap & parameters)
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
      "tws_battery.connect_failed",
      error_message,
      LifecycleState::kUnconfigured);
    return {TransitionResult::kError, error_message};
  }

  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome TwsBatteryRuntime::activate()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kInactive || !driver_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    if (!driver_->is_configured()) {
      return {TransitionResult::kFailure, "battery driver is not configured"};
    }
    state_ = LifecycleState::kActive;
  }
  start_polling();
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome TwsBatteryRuntime::deactivate()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kActive || !driver_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    state_ = LifecycleState::kInactive;
  }
  stop_polling();
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome TwsBatteryRuntime::cleanup(const ParameterMap &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LifecycleState::kInactive || !driver_) {
    return {TransitionResult::kFailure, "source state does not match"};
  }
  state_ = LifecycleState::kCleaningUp;
  driver_->cleanup();
  state_ = LifecycleState::kUnconfigured;
  return {TransitionResult::kSuccess, {}};
}

BatteryDriverConfig TwsBatteryRuntime::config_from(
  const ParameterMap & parameters,
  std::string & error_message) const
{
  auto config = driver_->config();
  if (const auto value = string_parameter(
      parameters, "device.interface.can_name", error_message))
  {
    config.can_interface = *value;
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(parameters, "battery.can_id", error_message)) {
    config.can_id = static_cast<uint32_t>(*value);
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = bool_parameter(
      parameters, "battery.use_extended_frame", error_message))
  {
    config.use_extended_frame = *value;
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(parameters, "request_timeout_ms", error_message)) {
    config.timeout_ms = static_cast<int>(*value);
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(parameters, "poll_interval_ms", error_message)) {
    config.poll_interval_ms = static_cast<int>(*value);
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(parameters, "identity_poll_divider", error_message)) {
    config.identity_poll_divider = static_cast<int>(*value);
  }
  if (!error_message.empty()) {
    return config;
  }
  if (const auto value = integer_parameter(
      parameters, "battery.device_address", error_message))
  {
    config.device_address = static_cast<uint8_t>(*value);
  }
  return config;
}

void TwsBatteryRuntime::emit_event(device_manager::DeviceEvent event)
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

void TwsBatteryRuntime::emit_error_event(
  std::string code,
  std::string message,
  LifecycleState target_state)
{
  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = device_manager::EventLevel::kError;
  event.code = std::move(code);
  event.message = std::move(message);
  event.source = "tws_battery_driver";
  event.target_state = target_state;
  emit_event(std::move(event));
}

void TwsBatteryRuntime::emit_ok_event(std::string code, std::string message)
{
  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = device_manager::EventLevel::kOk;
  event.code = std::move(code);
  event.message = std::move(message);
  event.source = "tws_battery_driver";
  event.target_state = LifecycleState::kUnknown;
  emit_event(std::move(event));
}

void TwsBatteryRuntime::start_polling()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!output_node_) {
      return;
    }
  }
  std::lock_guard<std::mutex> lock(poll_mutex_);
  if (poll_thread_.joinable()) {
    return;
  }
  stop_polling_ = false;
  poll_thread_ = std::thread([this]() {poll_loop();});
}

void TwsBatteryRuntime::stop_polling()
{
  {
    std::lock_guard<std::mutex> lock(poll_mutex_);
    stop_polling_ = true;
  }
  poll_condition_.notify_all();
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }
}

void TwsBatteryRuntime::poll_loop()
{
  std::unique_lock<std::mutex> lock(poll_mutex_);
  while (!stop_polling_) {
    const auto interval = std::chrono::milliseconds(poll_interval_ms());
    if (poll_condition_.wait_for(lock, interval, [this]() {return stop_polling_;})) {
      break;
    }
    lock.unlock();
    const bool ok = poll_once();
    const auto current = snapshot();
    std::string last_poll_error;
    {
      std::lock_guard<std::mutex> state_lock(mutex_);
      last_poll_error = last_poll_error_;
    }
    publish_snapshot(current);
    if (!ok) {
      RCLCPP_WARN_THROTTLE(
        output_node_->get_logger(), *output_node_->get_clock(), 5000,
        "电池轮询失败: %s",
        last_poll_error.empty() ? current.last_error.c_str() : last_poll_error.c_str());
    }
    lock.lock();
  }
}

void TwsBatteryRuntime::publish_snapshot(const BatterySnapshot & snapshot)
{
  battery_state_publisher_->publish(build_battery_state_msg(snapshot));
  bms_status_publisher_->publish(build_bms_status_msg(snapshot));
}

sensor_msgs::msg::BatteryState TwsBatteryRuntime::build_battery_state_msg(
  const BatterySnapshot & snapshot) const
{
  const BatteryStateProjection projection = project_battery_state(
    snapshot.connected, snapshot.work_state, snapshot.soc);

  sensor_msgs::msg::BatteryState message;
  message.header.stamp = output_node_->now();
  message.header.frame_id = "battery";
  message.present = snapshot.connected;
  message.voltage = snapshot.pack_voltage_v;
  message.current = snapshot.pack_current_a;
  message.percentage = snapshot.soc / 100.0F;
  message.charge = snapshot.remain_capacity_mah / kMilliToUnit;
  message.capacity = snapshot.remain_capacity_mah / kMilliToUnit;
  message.design_capacity = std::numeric_limits<float>::quiet_NaN();
  message.temperature =
    (snapshot.max_cell_temperature_c + snapshot.min_cell_temperature_c) / 2.0F;
  message.power_supply_status = projection.power_supply_status;
  message.power_supply_health = snapshot.protect_status == 0U ?
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD :
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
  return message;
}

tws_battery_driver_ros2::msg::BmsStatus TwsBatteryRuntime::build_bms_status_msg(
  const BatterySnapshot & snapshot) const
{
  const BatteryStateProjection projection = project_battery_state(
    snapshot.connected, snapshot.work_state, snapshot.soc);

  tws_battery_driver_ros2::msg::BmsStatus message;
  message.header.stamp = output_node_->now();
  message.header.frame_id = "battery";
  message.connected = snapshot.connected;
  message.last_error = snapshot.last_error;
  message.device_address = snapshot.device_address;
  message.work_state = projection.work_state;
  message.work_state_text = battery_work_state_to_text(projection.work_state);
  message.pack_voltage_v = snapshot.pack_voltage_v;
  message.pack_current_a = snapshot.pack_current_a;
  message.max_cell_voltage_v = snapshot.max_cell_voltage_v;
  message.min_cell_voltage_v = snapshot.min_cell_voltage_v;
  message.max_cell_temperature_c = snapshot.max_cell_temperature_c;
  message.min_cell_temperature_c = snapshot.min_cell_temperature_c;
  message.max_board_temperature_c = snapshot.max_board_temperature_c;
  message.min_board_temperature_c = snapshot.min_board_temperature_c;
  message.soc = snapshot.soc;
  message.soh = snapshot.soh;
  message.remain_capacity_mah = snapshot.remain_capacity_mah;
  message.cycle_count = snapshot.cycle_count;
  message.protect_status = snapshot.protect_status;
  message.active_protections = decode_protection_bits(snapshot.protect_status);
  message.io_status = snapshot.io_status;
  message.charge_mos_control = (snapshot.io_status & (1U << 0)) != 0U;
  message.discharge_mos_control = (snapshot.io_status & (1U << 1)) != 0U;
  message.charge_mos_state = (snapshot.io_status & (1U << 2)) != 0U;
  message.discharge_mos_state = (snapshot.io_status & (1U << 3)) != 0U;
  message.serial_number = snapshot.serial_number;
  message.software_version = snapshot.software_version;
  message.hardware_version = snapshot.hardware_version;
  return message;
}

}  // namespace tws_battery_driver_ros2

PLUGINLIB_EXPORT_CLASS(
  tws_battery_driver_ros2::TwsBatteryRuntime,
  device_manager::IDeviceRuntime)
