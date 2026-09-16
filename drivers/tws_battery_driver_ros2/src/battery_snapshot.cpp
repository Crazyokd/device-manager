#include "tws_battery_driver_ros2/battery_snapshot.hpp"

#include <iomanip>
#include <sstream>
#include <utility>

namespace tws_battery_driver_ros2
{

std::string sanitize_identity_string(const std::string & raw_value)
{
  std::string sanitized;
  sanitized.reserve(raw_value.size());

  for (const unsigned char value : raw_value) {
    if (value == '\0' || value == 0xFFU) {
      continue;
    }
    if (value >= 0x20U && value <= 0x7EU) {
      sanitized.push_back(static_cast<char>(value));
    }
  }

  const auto first = sanitized.find_first_not_of(' ');
  if (first == std::string::npos) {
    return "";
  }
  if (first > 0U) {
    sanitized.erase(0U, first);
  }

  while (!sanitized.empty() && sanitized.back() == ' ') {
    sanitized.pop_back();
  }

  return sanitized;
}

float decode_temperature_c(uint16_t raw_value)
{
  return static_cast<float>(raw_value) - 40.0F;
}

std::string version_to_text(uint16_t raw_value)
{
  std::ostringstream stream;
  stream << 'V' << static_cast<unsigned int>((raw_value >> 8) & 0xFFU) << '.'
         << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(raw_value & 0xFFU);
  return stream.str();
}

std::vector<std::string> decode_protection_bits(uint32_t mask)
{
  static const std::vector<std::pair<uint32_t, std::string>> kProtectionBitMap {
    {0x00000001U, "CHARGE_OVER_CURRENT"},
    {0x00000002U, "CHARGE_OVER_VOLTAGE"},
    {0x00000004U, "CHARGE_SINGLE_CELL_OVER_VOLTAGE"},
    {0x00000008U, "CHARGE_OVER_TEMPERATURE"},
    {0x00000010U, "CHARGE_UNDER_TEMPERATURE"},
    {0x00000020U, "DISCHARGE_OVER_CURRENT"},
    {0x00000040U, "BMS_SUM_UNDER_VOLTAGE"},
    {0x00000080U, "DISCHARGE_CELL_UNDER_VOLTAGE"},
    {0x00000100U, "DISCHARGE_OVER_TEMPERATURE"},
    {0x00000200U, "DISCHARGE_UNDER_TEMPERATURE"},
    {0x00000400U, "AFE_OVER_CURRENT"},
    {0x00000800U, "AFE_SOC_OVER_VOLTAGE"},
    {0x00001000U, "AFE_UNDER_VOLTAGE"},
    {0x00002000U, "MOS_OVER_TEMPERATURE"},
    {0x00008000U, "SOC_LOW_PROTECT"},
    {0x00010000U, "CELLS_IMBALANCE"},
    {0x00020000U, "CELLS_TEMP_DEVIATION"},
    {0x08000000U, "SECOND_LEVEL_OVER_TEMPERATURE"},
    {0x10000000U, "SECOND_LEVEL_CELL_OVER_VOLTAGE"},
    {0x20000000U, "SECOND_LEVEL_CELL_UNDER_VOLTAGE"},
    {0x40000000U, "SECOND_LEVEL_CHARGE_OVER_CURRENT"},
    {0x80000000U, "SECOND_LEVEL_DISCHARGE_OVER_CURRENT"}
  };

  std::vector<std::string> results;
  for (const auto & item : kProtectionBitMap) {
    if ((mask & item.first) != 0U) {
      results.push_back(item.second);
    }
  }
  return results;
}

}  // namespace tws_battery_driver_ros2
