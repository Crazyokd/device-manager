#ifndef TWS_BATTERY_DRIVER_ROS2__BATTERY_REGISTER_BLOCKS_HPP_
#define TWS_BATTERY_DRIVER_ROS2__BATTERY_REGISTER_BLOCKS_HPP_

#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace tws_battery_driver_ros2
{

inline constexpr uint16_t kFastBlockAStart = 0x9000U;
inline constexpr uint16_t kFastBlockACount = 0x0016U;
inline constexpr uint16_t kFastBlockBStart = 0x9028U;
inline constexpr uint16_t kFastBlockBCount = 0x0006U;
inline constexpr uint16_t kIdentityBlockStart = 0x9016U;
inline constexpr uint16_t kIdentityBlockCount = 0x0012U;

struct BatteryFastRegisterValues
{
  uint16_t work_state {0U};
  uint32_t pack_voltage_mv {0U};
  int32_t pack_current_ma {0};
  uint16_t max_cell_voltage_mv {0U};
  uint16_t min_cell_voltage_mv {0U};
  uint16_t max_cell_temp {0U};
  uint16_t min_cell_temp {0U};
  uint16_t max_board_temp {0U};
  uint16_t min_board_temp {0U};
  uint32_t protect_status {0U};
  uint32_t io_status {0U};
  uint16_t soc {0U};
  uint16_t soh {0U};
  uint32_t remain_capacity_mah {0U};
  uint32_t cycle_count {0U};
};

struct BatteryIdentityRegisterValues
{
  std::string serial_number_bytes;
  uint16_t software_version {0U};
  uint16_t hardware_version {0U};
};

inline uint16_t decode_block_u16(
  const std::vector<uint8_t> & bytes,
  std::size_t offset)
{
  return static_cast<uint16_t>(bytes[offset]) |
         static_cast<uint16_t>(bytes[offset + 1U] << 8U);
}

inline uint32_t decode_block_u32(
  const std::vector<uint8_t> & bytes,
  std::size_t offset)
{
  return static_cast<uint32_t>(decode_block_u16(bytes, offset)) |
         (static_cast<uint32_t>(decode_block_u16(bytes, offset + 2U)) << 16U);
}

inline bool decode_fast_register_blocks(
  const std::vector<uint8_t> & block_a,
  const std::vector<uint8_t> & block_b,
  BatteryFastRegisterValues & values,
  std::string & error_message)
{
  constexpr std::size_t kBlockABytes = static_cast<std::size_t>(kFastBlockACount) * 2U;
  constexpr std::size_t kBlockBBytes = static_cast<std::size_t>(kFastBlockBCount) * 2U;

  if (block_a.size() != kBlockABytes) {
    std::ostringstream stream;
    stream << "快速寄存器块 A 长度异常，期望 " << kBlockABytes <<
      " 字节，实际 " << block_a.size() << " 字节";
    error_message = stream.str();
    return false;
  }
  if (block_b.size() != kBlockBBytes) {
    std::ostringstream stream;
    stream << "快速寄存器块 B 长度异常，期望 " << kBlockBBytes <<
      " 字节，实际 " << block_b.size() << " 字节";
    error_message = stream.str();
    return false;
  }

  values.work_state = decode_block_u16(block_a, 0U);
  values.pack_voltage_mv = decode_block_u32(block_a, 4U);
  values.pack_current_ma = static_cast<int32_t>(decode_block_u32(block_a, 8U));
  values.max_cell_voltage_mv = decode_block_u16(block_a, 12U);
  values.min_cell_voltage_mv = decode_block_u16(block_a, 14U);
  values.max_cell_temp = decode_block_u16(block_a, 16U);
  values.min_cell_temp = decode_block_u16(block_a, 18U);
  values.max_board_temp = decode_block_u16(block_a, 20U);
  values.min_board_temp = decode_block_u16(block_a, 22U);
  values.protect_status = decode_block_u32(block_a, 24U);
  values.io_status = decode_block_u32(block_a, 40U);
  values.soc = decode_block_u16(block_b, 0U);
  values.soh = decode_block_u16(block_b, 2U);
  values.remain_capacity_mah = decode_block_u32(block_b, 4U);
  values.cycle_count = decode_block_u32(block_b, 8U);
  error_message.clear();
  return true;
}

inline bool decode_identity_register_block(
  const std::vector<uint8_t> & block,
  BatteryIdentityRegisterValues & values,
  std::string & error_message)
{
  constexpr std::size_t kIdentityBlockBytes =
    static_cast<std::size_t>(kIdentityBlockCount) * 2U;
  if (block.size() != kIdentityBlockBytes) {
    std::ostringstream stream;
    stream << "身份寄存器块长度异常，期望 " << kIdentityBlockBytes <<
      " 字节，实际 " << block.size() << " 字节";
    error_message = stream.str();
    return false;
  }

  values.serial_number_bytes.assign(
    reinterpret_cast<const char *>(block.data()), 32U);
  values.software_version = decode_block_u16(block, 32U);
  values.hardware_version = decode_block_u16(block, 34U);
  error_message.clear();
  return true;
}

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__BATTERY_REGISTER_BLOCKS_HPP_
