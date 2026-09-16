#ifndef TWS_BATTERY_DRIVER_ROS2__MODBUS_RESPONSE_ASSEMBLER_HPP_
#define TWS_BATTERY_DRIVER_ROS2__MODBUS_RESPONSE_ASSEMBLER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace tws_battery_driver_ros2
{

enum class ModbusResponseAssemblyStatus
{
  kIncomplete,
  kComplete,
  kError,
};

class ModbusResponseAssembler
{
public:
  ModbusResponseAssemblyStatus append(
    const std::vector<uint8_t> & fragment,
    std::string & error_message)
  {
    if (fragment.empty()) {
      error_message = "收到空 CAN 响应分片";
      return ModbusResponseAssemblyStatus::kError;
    }

    bytes_.insert(bytes_.end(), fragment.begin(), fragment.end());
    update_expected_size();
    if (!expected_size_.has_value()) {
      return ModbusResponseAssemblyStatus::kIncomplete;
    }
    if (bytes_.size() > *expected_size_) {
      std::ostringstream stream;
      stream << "Modbus 多帧响应长度超限，期望 " << *expected_size_ <<
        " 字节，实际 " << bytes_.size() << " 字节";
      error_message = stream.str();
      return ModbusResponseAssemblyStatus::kError;
    }
    if (bytes_.size() == *expected_size_) {
      error_message.clear();
      return ModbusResponseAssemblyStatus::kComplete;
    }
    return ModbusResponseAssemblyStatus::kIncomplete;
  }

  const std::vector<uint8_t> & bytes() const
  {
    return bytes_;
  }

  std::size_t expected_size() const
  {
    return expected_size_.value_or(0U);
  }

  std::string timeout_error() const
  {
    if (bytes_.empty()) {
      return "等待 BMS 响应超时";
    }

    std::ostringstream stream;
    stream << "等待 BMS 响应超时，已收到 " << bytes_.size() << " 字节";
    if (expected_size_.has_value()) {
      stream << "，期望 " << *expected_size_ << " 字节";
    }
    return stream.str();
  }

private:
  void update_expected_size()
  {
    if (expected_size_.has_value() || bytes_.size() < 2U) {
      return;
    }
    if (bytes_[1] == 0x83U) {
      expected_size_ = 5U;
    } else if (bytes_.size() >= 3U) {
      expected_size_ = static_cast<std::size_t>(bytes_[2]) + 5U;
    }
  }

  std::vector<uint8_t> bytes_;
  std::optional<std::size_t> expected_size_;
};

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__MODBUS_RESPONSE_ASSEMBLER_HPP_
