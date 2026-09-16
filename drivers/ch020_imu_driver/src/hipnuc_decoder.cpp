#include "ch020_imu_driver/hipnuc_decoder.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <utility>

namespace ch020_imu_driver
{

namespace
{

constexpr uint8_t kSof1 = 0x5A;
constexpr uint8_t kSof2 = 0xA5;
constexpr uint8_t kTagImuSol = 0x91;
constexpr uint16_t kImuSolLen = 76;
constexpr float kG = 9.80665F;
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0F;

}  // namespace

// ---------------------------------------------------------------------------
// CRC-16/CCITT（多项式 0x1021，初值 0）——与手册 §6 C 实现等价。
// ---------------------------------------------------------------------------
uint16_t HipnucDecoder::crc16_update(uint16_t crc, const uint8_t * data, std::size_t length)
{
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int j = 0; j < 8; ++j) {
      uint16_t t = static_cast<uint16_t>(crc << 1);
      if ((crc & 0x8000U) != 0U) {
        t ^= 0x1021U;
      }
      crc = t;
    }
  }
  return crc;
}

// ---------------------------------------------------------------------------

HipnucDecoder::HipnucDecoder(PacketCallback callback)
: callback_(std::move(callback))
{
}

void HipnucDecoder::feed(const uint8_t * data, std::size_t length)
{
  for (std::size_t i = 0; i < length; ++i) {
    const uint8_t b = data[i];
    switch (state_) {
      case State::SOF1:
        if (b == kSof1) {
          state_ = State::SOF2;
        }
        break;
      case State::SOF2:
        state_ = (b == kSof2) ? State::LEN1 : State::SOF1;
        break;
      case State::LEN1:
        payload_len_ = b;
        state_ = State::LEN2;
        break;
      case State::LEN2:
        payload_len_ |= static_cast<uint16_t>(b) << 8;
        buf_.clear();
        buf_.reserve(payload_len_);
        state_ = State::CRC1;
        break;
      case State::CRC1:
        crc_lo_ = b;
        state_ = State::CRC2;
        break;
      case State::CRC2:
        crc_rx_ = crc_lo_ | (static_cast<uint16_t>(b) << 8);
        if (payload_len_ == 0) {
          process_frame();
        } else {
          state_ = State::DATA;
        }
        break;
      case State::DATA:
        buf_.push_back(b);
        if (buf_.size() == payload_len_) {
          process_frame();
        }
        break;
    }
  }
}

void HipnucDecoder::process_frame()
{
  // CRC 覆盖：[0x5A, 0xA5, LEN_L, LEN_H] + PAYLOAD
  const std::array<uint8_t, 4> header = {
    kSof1, kSof2,
    static_cast<uint8_t>(payload_len_ & 0xFFU),
    static_cast<uint8_t>((payload_len_ >> 8) & 0xFFU)
  };
  uint16_t expected = crc16_update(0, header.data(), 4);
  expected = crc16_update(expected, buf_.data(), buf_.size());

  const auto payload = std::move(buf_);
  buf_ = {};
  state_ = State::SOF1;
  payload_len_ = 0;

  if (expected != crc_rx_) {
    return;
  }
  if (payload.size() < kImuSolLen || payload[0] != kTagImuSol) {
    return;
  }

  const uint8_t * p = payload.data();

  auto read_f32 = [&](std::size_t offset) noexcept {
      float value;
      std::memcpy(&value, p + offset, sizeof(float));
      return value;
    };
  auto read_u32 = [&](std::size_t offset) noexcept {
      uint32_t value;
      std::memcpy(&value, p + offset, sizeof(uint32_t));
      return value;
    };

  ImuSolPacket packet {};
  packet.device_id = p[1];
  packet.temperature = static_cast<float>(static_cast<int8_t>(p[3]));
  packet.pressure = read_f32(4);
  packet.timestamp_ms = read_u32(8);
  packet.acc_x = read_f32(12) * kG;
  packet.acc_y = read_f32(16) * kG;
  packet.acc_z = read_f32(20) * kG;
  packet.gyro_x = read_f32(24) * kDegToRad;
  packet.gyro_y = read_f32(28) * kDegToRad;
  packet.gyro_z = read_f32(32) * kDegToRad;
  packet.mag_x = read_f32(36);
  packet.mag_y = read_f32(40);
  packet.mag_z = read_f32(44);
  packet.roll = read_f32(48);
  packet.pitch = read_f32(52);
  packet.yaw = read_f32(56);
  packet.quat_w = read_f32(60);
  packet.quat_x = read_f32(64);
  packet.quat_y = read_f32(68);
  packet.quat_z = read_f32(72);

  callback_(packet);
}

}  // namespace ch020_imu_driver
