#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "tws_battery_driver_ros2/modbus_response_assembler.hpp"

namespace tws_battery_driver_ros2
{
namespace
{

TEST(ModbusResponseAssembler, JoinsNineByteResponseFromEightAndOne)
{
  ModbusResponseAssembler assembler;
  std::string error;

  EXPECT_EQ(
    assembler.append({0x01, 0x03, 0x04, 0x42, 0x00, 0x64, 0x00, 0xC4}, error),
    ModbusResponseAssemblyStatus::kIncomplete);
  EXPECT_EQ(
    assembler.append({0x8B}, error),
    ModbusResponseAssemblyStatus::kComplete);
  EXPECT_EQ(
    assembler.bytes(),
    (std::vector<uint8_t> {0x01, 0x03, 0x04, 0x42, 0x00, 0x64, 0x00, 0xC4, 0x8B}));
}

TEST(ModbusResponseAssembler, JoinsSeventeenByteResponseFromThreeFrames)
{
  ModbusResponseAssembler assembler;
  std::string error;

  EXPECT_EQ(
    assembler.append({0x01, 0x03, 0x0C, 0x42, 0x00, 0x64, 0x00, 0x51}, error),
    ModbusResponseAssemblyStatus::kIncomplete);
  EXPECT_EQ(
    assembler.append({0x26, 0x00, 0x00, 0x17, 0x00, 0x00, 0x00, 0xAF}, error),
    ModbusResponseAssemblyStatus::kIncomplete);
  EXPECT_EQ(
    assembler.append({0x65}, error),
    ModbusResponseAssemblyStatus::kComplete);
  EXPECT_EQ(assembler.bytes().size(), 17U);
}

TEST(ModbusResponseAssembler, JoinsFortyNineByteResponseFromSevenFrames)
{
  const std::vector<uint8_t> response {
    0x01, 0x03, 0x2C,
    0x02, 0x00, 0x00, 0x00, 0xB5, 0xCF, 0x00, 0x00,
    0x94, 0xFD, 0xFF, 0xFF, 0xFE, 0x0C, 0xF9, 0x0C,
    0x48, 0x00, 0x47, 0x00, 0x47, 0x00, 0x46, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0F, 0x80, 0x10, 0x40, 0x3A, 0xFB,
  };
  ModbusResponseAssembler assembler;
  std::string error;

  for (std::size_t offset = 0U; offset < response.size(); offset += 8U) {
    const std::size_t end = std::min(offset + 8U, response.size());
    const std::vector<uint8_t> fragment(response.begin() + offset, response.begin() + end);
    const auto status = assembler.append(fragment, error);
    EXPECT_EQ(
      status,
      end == response.size() ?
      ModbusResponseAssemblyStatus::kComplete :
      ModbusResponseAssemblyStatus::kIncomplete);
  }
  EXPECT_EQ(assembler.bytes(), response);
}

TEST(ModbusResponseAssembler, HandlesFiveByteExceptionResponse)
{
  ModbusResponseAssembler assembler;
  std::string error;

  EXPECT_EQ(
    assembler.append({0x01, 0x83, 0x02, 0xC0, 0xF1}, error),
    ModbusResponseAssemblyStatus::kComplete);
  EXPECT_EQ(assembler.expected_size(), 5U);
}

TEST(ModbusResponseAssembler, RejectsDuplicateFragmentThatExceedsExpectedSize)
{
  ModbusResponseAssembler assembler;
  std::string error;
  const std::vector<uint8_t> first {0x01, 0x03, 0x04, 0x42, 0x00, 0x64, 0x00, 0xC4};

  ASSERT_EQ(
    assembler.append(first, error),
    ModbusResponseAssemblyStatus::kIncomplete);
  EXPECT_EQ(
    assembler.append(first, error),
    ModbusResponseAssemblyStatus::kError);
  EXPECT_EQ(error, "Modbus 多帧响应长度超限，期望 9 字节，实际 16 字节");
}

TEST(ModbusResponseAssembler, DescribesPartialTimeout)
{
  ModbusResponseAssembler assembler;
  std::string error;
  ASSERT_EQ(
    assembler.append({0x01, 0x03, 0x0C, 0x42, 0x00, 0x64, 0x00, 0x51}, error),
    ModbusResponseAssemblyStatus::kIncomplete);

  EXPECT_EQ(
    assembler.timeout_error(),
    "等待 BMS 响应超时，已收到 8 字节，期望 17 字节");
}

}  // namespace
}  // namespace tws_battery_driver_ros2
