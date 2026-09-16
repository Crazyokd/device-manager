#include "smit_ros_driver/smit_protocol.hpp"

#include <algorithm>
#include <utility>

namespace smit_ros_driver
{

namespace
{

constexpr uint8_t kSof1 = 0xA5;
constexpr uint8_t kSof2 = 0x5A;
constexpr uint8_t kMsgIdPointCloud = 0x01;
constexpr uint8_t kMsgIdConfig = 0x02;
constexpr std::size_t kHeaderLength = 4;
constexpr std::size_t kMaxFrameLength = 2048;     ///< 超出视为异常长度，丢帧重同步
constexpr std::size_t kMaxBufferLength = 10000;   ///< 缓冲区过载保护
constexpr std::size_t kConfigPayloadMinLength = 33;  ///< msg_id + 31 字节配置 + CRC8
constexpr float kDistanceScale = 0.0025F;         ///< 距离分辨率 m
constexpr double kDeltaAngleMin = 0.172;          ///< delta_angle 合法区间（度）
constexpr double kDeltaAngleMax = 0.188;

uint16_t read_u16_be(const uint8_t * p)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

uint32_t read_u24_be(const uint8_t * p)
{
  return (static_cast<uint32_t>(p[0]) << 16) |
         (static_cast<uint32_t>(p[1]) << 8) |
         static_cast<uint32_t>(p[2]);
}

}  // namespace

uint8_t SmitProtocol::crc8(const uint8_t * data, std::size_t length)
{
  uint8_t crc = 0x00;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) != 0U ? static_cast<uint8_t>((crc << 1) ^ 0x07U) :
        static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

SmitProtocol::SmitProtocol(ConfigCallback config_callback, PointCallback point_callback)
: config_callback_(std::move(config_callback)),
  point_callback_(std::move(point_callback))
{
}

void SmitProtocol::feed(const uint8_t * data, std::size_t length)
{
  buffer_.insert(buffer_.end(), data, data + length);
  if (buffer_.size() > kMaxBufferLength) {
    buffer_.clear();
    return;
  }

  while (buffer_.size() >= kHeaderLength) {
    if (buffer_[0] != kSof1 || buffer_[1] != kSof2) {
      buffer_.erase(buffer_.begin());
      continue;
    }

    const uint16_t payload_len = read_u16_be(buffer_.data() + 2);
    const std::size_t total_len = static_cast<std::size_t>(payload_len) + kHeaderLength;
    if (total_len > kMaxFrameLength) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + 2);
      continue;
    }
    if (buffer_.size() < total_len) {
      break;  // 半包，等下一轮数据
    }

    process_frame(buffer_.data(), total_len);
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_len);
  }
}

void SmitProtocol::process_frame(const uint8_t * frame, std::size_t length)
{
  const uint8_t * payload = frame + kHeaderLength;
  const std::size_t payload_len = length - kHeaderLength;
  if (payload_len < 2) {  // 至少 msg_id + CRC8
    return;
  }

  const bool crc_ok = crc8(payload, payload_len - 1) == payload[payload_len - 1];
  switch (payload[0]) {
    case kMsgIdPointCloud:
      parse_points(payload, payload_len, crc_ok);
      break;
    case kMsgIdConfig:
      if (crc_ok) {
        parse_config(payload, payload_len);
      }
      break;
    default:
      break;
  }
}

void SmitProtocol::parse_config(const uint8_t * payload, std::size_t length)
{
  if (length < kConfigPayloadMinLength) {
    return;
  }

  SmitConfigInfo config;
  std::copy(payload + 1, payload + 15, config.sn.begin());
  std::copy(payload + 15, payload + 18, config.hw_ver.begin());
  std::copy(payload + 18, payload + 21, config.fpga_ver.begin());
  std::copy(payload + 21, payload + 24, config.mcu_ver.begin());
  config.motor_speed = payload[25] == 3 ? 20 : payload[25] == 2 ? 15 : 10;
  config.instant_speed = read_u16_be(payload + 26);
  config.lidar_freq = payload[28] == 0 ? 10000 : payload[28] == 1 ? 20000 : 40000;
  config.lidar_temp = payload[29];
  config.lidar_volt = static_cast<float>(payload[30]) / 10.0F;
  config.lidar_state = payload[31];
  config_callback_(config);
}

void SmitProtocol::parse_points(const uint8_t * payload, std::size_t length, bool crc_ok)
{
  SmitPointPacket packet;
  packet.crc_ok = crc_ok;
  if (!crc_ok) {
    // 坏包上抛，供扫描装配器丢弃积累的半圈数据
    point_callback_(packet);
    return;
  }
  if (length <= 9) {
    return;
  }

  const std::size_t point_count = (length - 9) / 3;
  if (point_count > SmitPointPacket::kMaxPoints) {
    return;
  }

  const float start_angle = static_cast<float>(read_u16_be(payload + 1)) * 0.01F;
  const float delta_angle = static_cast<float>(read_u16_be(payload + 3)) * 0.0001F;
  if (delta_angle < kDeltaAngleMin || delta_angle > kDeltaAngleMax) {
    return;
  }

  packet.timestamp = read_u24_be(payload + length - 4);
  const uint8_t * point_data = payload + 5;
  for (std::size_t i = 0; i < point_count; ++i) {
    SmitPoint point;
    point.intensity = static_cast<float>(point_data[i * 3]);
    point.distance_m = static_cast<float>(read_u16_be(point_data + i * 3 + 1)) * kDistanceScale;
    const float angle = start_angle + delta_angle * static_cast<float>(i);
    point.angle_deg = angle > 360.0F ? angle - 360.0F : angle;
    packet.points[i] = point;
  }
  packet.point_count = point_count;
  point_callback_(packet);
}

}  // namespace smit_ros_driver
