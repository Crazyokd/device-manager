#include "pager100_lora_bridge/pager100_lora_runtime.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace pager100_lora_bridge
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

Pager100LoraRuntime::Pager100LoraRuntime(
  Pager100LoraConfig config,
  Pager100LoraDriver::TransportFactory transport_factory)
: config_(std::move(config)),
  transport_factory_(std::move(transport_factory)),
  state_(LifecycleState::kFinalized)
{
}

Pager100LoraRuntime::~Pager100LoraRuntime()
{
  stop_output_spin();
}

void Pager100LoraRuntime::initialize(const device_manager::DeviceDefinition &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_handler_) {
    return;
  }

  output_node_ = std::make_shared<rclcpp::Node>(
    "pager100_lora_runtime", rclcpp::NodeOptions().use_global_arguments(false));
  rx_publisher_ = output_node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
    "/pager100/lora/rx", 10);
  tx_subscription_ = output_node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/pager100/lora/tx", 10,
    [this](const std_msgs::msg::UInt8MultiArray & message) {on_tx(message);});
  frame_handler_ = [this](const std::vector<std::uint8_t> & payload) {
      std_msgs::msg::UInt8MultiArray message;
      message.data = payload;
      rx_publisher_->publish(message);
    };

  // runtime 自建 node 不在宿主的 executor 里；tx 订阅需要一个专用 spin 线程。
  // executor cancel 只停本 executor，不碰宿主共享的全局 context。
  output_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  output_executor_->add_node(output_node_);
  output_spin_thread_ = std::thread([this]() {output_executor_->spin();});
}

LifecycleState Pager100LoraRuntime::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

TransitionOutcome Pager100LoraRuntime::materialize(const ParameterMap &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LifecycleState::kFinalized || driver_) {
    return {TransitionResult::kFailure, "runtime is already materialized"};
  }
  driver_ = std::make_shared<Pager100LoraDriver>(config_, transport_factory_);
  driver_->set_frame_handler(frame_handler_);
  driver_->set_connection_handler(
    [this](bool connected, const std::string & message) {
      if (connected) {
        emit_ok_event("pager100.online", message);
      } else {
        emit_error_event("pager100.disconnected", message, LifecycleState::kActive);
      }
    });
  state_ = LifecycleState::kUnconfigured;
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome Pager100LoraRuntime::dematerialize()
{
  std::shared_ptr<Pager100LoraDriver> driver;
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

void Pager100LoraRuntime::set_event_handler(device_manager::EventHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  event_handler_ = std::move(handler);
}

void Pager100LoraRuntime::register_hook(device_manager::HookRegistrar registrar)
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

void Pager100LoraRuntime::set_frame_handler(Pager100LoraDriver::FrameHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  frame_handler_ = std::move(handler);
  if (driver_) {
    driver_->set_frame_handler(frame_handler_);
  }
}

Pager100LoraSnapshot Pager100LoraRuntime::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return driver_ ? driver_->snapshot() : Pager100LoraSnapshot{};
}

TransitionOutcome Pager100LoraRuntime::configure(const ParameterMap & parameters)
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
      "pager100.connect_failed",
      error_message,
      LifecycleState::kUnconfigured);
    return {TransitionResult::kError, error_message};
  }

  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome Pager100LoraRuntime::activate()
{
  Pager100LoraDriver * driver;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kInactive || !driver_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    if (!driver_->is_configured()) {
      return {TransitionResult::kFailure, "lora driver is not configured"};
    }
    state_ = LifecycleState::kActive;
    driver = driver_.get();
  }
  // start() 只启动读线程、不等待回调；在锁外调用，避免读线程事件回调持锁等待。
  driver->start();
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome Pager100LoraRuntime::deactivate()
{
  Pager100LoraDriver * driver;
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

TransitionOutcome Pager100LoraRuntime::cleanup(const ParameterMap &)
{
  Pager100LoraDriver * driver;
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

Pager100LoraConfig Pager100LoraRuntime::config_from(
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
  if (const auto value = integer_parameter(parameters, "reconnect_backoff_ms", error_message)) {
    config.reconnect_backoff_ms = static_cast<int>(*value);
  }
  return config;
}

void Pager100LoraRuntime::emit_event(device_manager::DeviceEvent event)
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

void Pager100LoraRuntime::emit_error_event(
  std::string code,
  std::string message,
  LifecycleState target_state)
{
  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = device_manager::EventLevel::kError;
  event.code = std::move(code);
  event.message = std::move(message);
  event.source = "pager100_lora_bridge";
  event.target_state = target_state;
  emit_event(std::move(event));
}

void Pager100LoraRuntime::emit_ok_event(std::string code, std::string message)
{
  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = device_manager::EventLevel::kOk;
  event.code = std::move(code);
  event.message = std::move(message);
  event.source = "pager100_lora_bridge";
  event.target_state = LifecycleState::kUnknown;
  emit_event(std::move(event));
}

void Pager100LoraRuntime::on_tx(const std_msgs::msg::UInt8MultiArray & message)
{
  // 与 dematerialize 并发时靠 shared_ptr 保活；cleanup 后的 send 会返回失败。
  std::shared_ptr<Pager100LoraDriver> driver;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    driver = driver_;
  }
  if (!driver) {
    return;
  }

  std::string error_message;
  if (!driver->send(message.data, error_message)) {
    RCLCPP_WARN_THROTTLE(
      output_node_->get_logger(), *output_node_->get_clock(), 1000,
      "LoRa 下发失败: %s", error_message.c_str());
  }
}

void Pager100LoraRuntime::stop_output_spin()
{
  if (output_executor_) {
    output_executor_->cancel();
  }
  if (output_spin_thread_.joinable()) {
    output_spin_thread_.join();
  }
}

}  // namespace pager100_lora_bridge

PLUGINLIB_EXPORT_CLASS(
  pager100_lora_bridge::Pager100LoraRuntime,
  device_manager::IDeviceRuntime)
