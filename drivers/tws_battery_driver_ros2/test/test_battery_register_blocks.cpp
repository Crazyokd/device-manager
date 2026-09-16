#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "tws_battery_driver_ros2/battery_register_blocks.hpp"

namespace tws_battery_driver_ros2
{
namespace
{

TEST(BatteryRegisterBlocks, DecodesFastBlocksFromMeasuredBytes)
{
  const std::vector<uint8_t> block_a {
    0x02, 0x00, 0x00, 0x00, 0xB5, 0xCF, 0x00, 0x00,
    0x94, 0xFD, 0xFF, 0xFF, 0xFE, 0x0C, 0xF9, 0x0C,
    0x48, 0x00, 0x47, 0x00, 0x47, 0x00, 0x46, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0F, 0x80, 0x10, 0x40,
  };
  const std::vector<uint8_t> block_b {
    0x42, 0x00, 0x64, 0x00, 0x51, 0x26,
    0x00, 0x00, 0x17, 0x00, 0x00, 0x00,
  };
  BatteryFastRegisterValues values;
  std::string error;

  ASSERT_TRUE(decode_fast_register_blocks(block_a, block_b, values, error));
  EXPECT_EQ(values.work_state, 2U);
  EXPECT_EQ(values.pack_voltage_mv, 53173U);
  EXPECT_EQ(values.pack_current_ma, -620);
  EXPECT_EQ(values.max_cell_voltage_mv, 3326U);
  EXPECT_EQ(values.min_cell_voltage_mv, 3321U);
  EXPECT_EQ(values.max_cell_temp, 72U);
  EXPECT_EQ(values.min_cell_temp, 71U);
  EXPECT_EQ(values.max_board_temp, 71U);
  EXPECT_EQ(values.min_board_temp, 70U);
  EXPECT_EQ(values.protect_status, 0U);
  EXPECT_EQ(values.io_status, 0x4010800FU);
  EXPECT_EQ(values.soc, 66U);
  EXPECT_EQ(values.soh, 100U);
  EXPECT_EQ(values.remain_capacity_mah, 9809U);
  EXPECT_EQ(values.cycle_count, 23U);
}

TEST(BatteryRegisterBlocks, RejectsWrongFastBlockSizes)
{
  BatteryFastRegisterValues values;
  std::string error;

  EXPECT_FALSE(decode_fast_register_blocks(
      std::vector<uint8_t>(43U), std::vector<uint8_t>(12U), values, error));
  EXPECT_EQ(error, "快速寄存器块 A 长度异常，期望 44 字节，实际 43 字节");

  EXPECT_FALSE(decode_fast_register_blocks(
      std::vector<uint8_t>(44U), std::vector<uint8_t>(11U), values, error));
  EXPECT_EQ(error, "快速寄存器块 B 长度异常，期望 12 字节，实际 11 字节");
}

TEST(BatteryRegisterBlocks, DecodesIdentityBlock)
{
  std::vector<uint8_t> block(36U, 0U);
  const std::string serial = "EG260701C0001";
  std::copy(serial.begin(), serial.end(), block.begin());
  block[32] = 0x64U;
  block[34] = 0x01U;
  BatteryIdentityRegisterValues values;
  std::string error;

  ASSERT_TRUE(decode_identity_register_block(block, values, error));
  EXPECT_EQ(values.serial_number_bytes.substr(0U, serial.size()), serial);
  EXPECT_EQ(values.software_version, 100U);
  EXPECT_EQ(values.hardware_version, 1U);
}

TEST(BatteryRegisterBlocks, RejectsWrongIdentityBlockSize)
{
  BatteryIdentityRegisterValues values;
  std::string error;

  EXPECT_FALSE(decode_identity_register_block(
      std::vector<uint8_t>(35U), values, error));
  EXPECT_EQ(error, "身份寄存器块长度异常，期望 36 字节，实际 35 字节");
}

}  // namespace
}  // namespace tws_battery_driver_ros2
