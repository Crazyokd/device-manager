#include "tws_battery_driver_ros2/tws_battery_driver.hpp"

#include <utility>

#include "tws_battery_driver_ros2/battery_register_blocks.hpp"
#include "tws_battery_driver_ros2/socketcan_transport.hpp"

namespace tws_battery_driver_ros2
{
namespace
{

constexpr float kMilliToUnit = 1000.0F;

std::unique_ptr<BatteryTransport> make_socketcan_transport(const BatteryDriverConfig & config)
{
  return std::make_unique<SocketCanTransport>(
    config.can_interface, config.can_id, config.use_extended_frame, config.timeout_ms);
}

}  // namespace

TwsBatteryDriver::TwsBatteryDriver(
  BatteryDriverConfig config,
  TransportFactory transport_factory)
: config_(std::move(config)),
  transport_factory_(
    transport_factory ? std::move(transport_factory) : TransportFactory(make_socketcan_transport))
{
  last_snapshot_.device_address = config_.device_address;
}

bool TwsBatteryDriver::configure(
  const BatteryDriverConfig & config,
  std::string & error_message)
{
  cleanup();
  config_ = config;
  last_snapshot_.device_address = config_.device_address;
  transport_ = transport_factory_(config_);
  if (!transport_) {
    error_message = "battery transport factory returned null";
    last_snapshot_.connected = false;
    last_snapshot_.last_error = error_message;
    return false;
  }
  return ensure_connected(error_message);
}

void TwsBatteryDriver::cleanup()
{
  if (transport_) {
    transport_->disconnect();
  }
  transport_.reset();
  has_successful_poll_ = false;
  consecutive_poll_failures_ = 0U;
  poll_counter_ = 0U;
}

bool TwsBatteryDriver::is_configured() const
{
  return static_cast<bool>(transport_);
}

BatteryPollResult TwsBatteryDriver::poll_once()
{
  BatteryPollResult result;
  result.snapshot = last_snapshot_;
  result.snapshot.device_address = config_.device_address;

  std::string error_message;
  if (!ensure_connected(error_message)) {
    return make_poll_failure_result(error_message);
  }

  if (!read_fast_snapshot(result.snapshot, error_message)) {
    return make_poll_failure_result(error_message);
  }

  if (config_.identity_poll_divider > 0 &&
    (poll_counter_ % static_cast<std::size_t>(config_.identity_poll_divider)) == 0U)
  {
    read_identity_snapshot(result.snapshot, result);
  }

  result.snapshot.connected = true;
  result.snapshot.last_error.clear();
  has_successful_poll_ = true;
  consecutive_poll_failures_ = 0U;
  append_status_log(
    status_transition_tracker_.observe_online(result.snapshot.work_state), result);
  append_soc_log(soc_transition_tracker_.observe(result.snapshot.soc), result);
  last_snapshot_ = result.snapshot;
  ++poll_counter_;
  result.ok = true;
  return result;
}

BatterySnapshot TwsBatteryDriver::snapshot() const
{
  return last_snapshot_;
}

const BatteryDriverConfig & TwsBatteryDriver::config() const
{
  return config_;
}

bool TwsBatteryDriver::ensure_connected(std::string & error_message)
{
  if (!transport_) {
    transport_ = transport_factory_(config_);
    if (!transport_) {
      error_message = "battery transport factory returned null";
      last_snapshot_.connected = false;
      last_snapshot_.last_error = error_message;
      return false;
    }
  }

  if (transport_->is_connected()) {
    error_message.clear();
    return true;
  }

  const bool ok = transport_->connect(error_message);
  if (!ok) {
    last_snapshot_.connected = false;
    last_snapshot_.last_error = error_message;
  }
  return ok;
}

bool TwsBatteryDriver::read_fast_snapshot(
  BatterySnapshot & snapshot,
  std::string & error_message)
{
  std::vector<uint8_t> block_a;
  std::vector<uint8_t> block_b;
  if (!transport_->read_holding_registers(
      config_.device_address, kFastBlockAStart, kFastBlockACount, block_a, error_message) ||
    !transport_->read_holding_registers(
      config_.device_address, kFastBlockBStart, kFastBlockBCount, block_b, error_message))
  {
    return false;
  }

  BatteryFastRegisterValues values;
  if (!decode_fast_register_blocks(block_a, block_b, values, error_message)) {
    return false;
  }

  snapshot.work_state = values.work_state;
  snapshot.pack_voltage_v = static_cast<float>(values.pack_voltage_mv) / kMilliToUnit;
  snapshot.pack_current_a = static_cast<float>(values.pack_current_ma) / kMilliToUnit;
  snapshot.max_cell_voltage_v = static_cast<float>(values.max_cell_voltage_mv) / kMilliToUnit;
  snapshot.min_cell_voltage_v = static_cast<float>(values.min_cell_voltage_mv) / kMilliToUnit;
  snapshot.max_cell_temperature_c = decode_temperature_c(values.max_cell_temp);
  snapshot.min_cell_temperature_c = decode_temperature_c(values.min_cell_temp);
  snapshot.max_board_temperature_c = decode_temperature_c(values.max_board_temp);
  snapshot.min_board_temperature_c = decode_temperature_c(values.min_board_temp);
  snapshot.protect_status = values.protect_status;
  snapshot.io_status = values.io_status;
  snapshot.soc = soc_filter_.filter(static_cast<float>(values.soc));
  snapshot.soh = static_cast<float>(values.soh);
  snapshot.remain_capacity_mah = static_cast<float>(values.remain_capacity_mah);
  snapshot.cycle_count = values.cycle_count;
  return true;
}

void TwsBatteryDriver::read_identity_snapshot(
  BatterySnapshot & snapshot,
  BatteryPollResult & result)
{
  std::string error_message;
  std::vector<uint8_t> block;
  if (!transport_->read_holding_registers(
      config_.device_address, kIdentityBlockStart, kIdentityBlockCount, block, error_message))
  {
    result.log_records.push_back(
      {BatteryStatusLogLevel::kWarn, "读取电池身份信息失败: " + error_message});
    return;
  }

  BatteryIdentityRegisterValues values;
  if (!decode_identity_register_block(block, values, error_message)) {
    result.log_records.push_back(
      {BatteryStatusLogLevel::kWarn, "解析电池身份信息失败: " + error_message});
    return;
  }

  snapshot.serial_number = sanitize_identity_string(values.serial_number_bytes);
  snapshot.software_version = version_to_text(values.software_version);
  snapshot.hardware_version = version_to_text(values.hardware_version);
}

BatteryPollResult TwsBatteryDriver::make_poll_failure_result(const std::string & error_message)
{
  BatteryPollResult result;
  result.error_message = error_message;
  result.snapshot = last_snapshot_;
  result.snapshot.device_address = config_.device_address;

  ++consecutive_poll_failures_;
  const bool has_recent_online_snapshot = last_snapshot_.connected;
  if (
    has_recent_online_snapshot &&
    consecutive_poll_failures_ < kConsecutivePollFailuresToOffline)
  {
    result.log_records.push_back(
      {BatteryStatusLogLevel::kWarn, "电池轮询失败，暂不判定掉线: " + error_message});
    return result;
  }

  result.snapshot.connected = false;
  result.snapshot.last_error = error_message;
  const auto transition = status_transition_tracker_.observe_offline(error_message);
  append_status_log(transition, result);
  result.report_failure_event = transition.has_value() || !has_successful_poll_;
  last_snapshot_ = result.snapshot;
  return result;
}

void TwsBatteryDriver::append_status_log(
  const std::optional<BatteryStatusTransitionEvent> & event,
  BatteryPollResult & result) const
{
  if (event) {
    result.log_records.push_back(make_battery_status_log_record(*event));
  }
}

void TwsBatteryDriver::append_soc_log(
  const std::optional<BatterySocTransitionEvent> & event,
  BatteryPollResult & result) const
{
  if (event) {
    result.log_records.push_back(make_battery_soc_log_record(*event));
  }
}

}  // namespace tws_battery_driver_ros2
