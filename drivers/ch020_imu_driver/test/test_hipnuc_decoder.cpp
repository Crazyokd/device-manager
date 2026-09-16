#include "ch020_imu_driver/hipnuc_decoder.hpp"

#include <array>
#include <cmath>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

using ch020_imu_driver::HipnucDecoder;
using ch020_imu_driver::ImuSolPacket;

// ---------------------------------------------------------------------------
// 测试辅助
// ---------------------------------------------------------------------------

static constexpr float G = 9.80665F;
static constexpr float D2R = static_cast<float>(M_PI) / 180.0F;

static uint16_t crc16(const uint8_t * data, std::size_t len)
{
  uint16_t crc = 0;
  for (std::size_t i = 0; i < len; ++i) {
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

static std::vector<uint8_t> build_frame(const std::vector<uint8_t> & payload)
{
  const uint8_t ll = static_cast<uint8_t>(payload.size() & 0xFFU);
  const uint8_t lh = static_cast<uint8_t>((payload.size() >> 8) & 0xFFU);

  std::vector<uint8_t> crc_in = {0x5A, 0xA5, ll, lh};
  crc_in.insert(crc_in.end(), payload.begin(), payload.end());

  const uint16_t crc = crc16(crc_in.data(), crc_in.size());
  std::vector<uint8_t> frame = {
    0x5A, 0xA5, ll, lh,
    static_cast<uint8_t>(crc & 0xFF),
    static_cast<uint8_t>(crc >> 8)};
  frame.insert(frame.end(), payload.begin(), payload.end());
  return frame;
}

/// 构造标准 76 字节 0x91 IMUSOL 载荷
static std::vector<uint8_t> make_imusol(
  uint8_t device_id = 0,
  int8_t temp = 25,
  float pressure = 0.0F,
  uint32_t ts = 0,
  std::array<float, 3> acc = {0.0F, 0.0F, 1.0F},   // G
  std::array<float, 3> gyr = {0.0F, 0.0F, 0.0F},   // deg/s
  std::array<float, 3> mag = {0.0F, 0.0F, 0.0F},   // µT
  std::array<float, 3> eul = {0.0F, 0.0F, 0.0F},   // deg
  std::array<float, 4> quat = {1.0F, 0.0F, 0.0F, 0.0F})
{
  std::vector<uint8_t> p(76, 0);
  p[0] = 0x91;
  p[1] = device_id;
  p[2] = 0;
  p[3] = static_cast<uint8_t>(static_cast<int8_t>(temp));
  auto wr_f32 = [&](std::size_t off, float v) {
      std::memcpy(p.data() + off, &v, 4);
    };
  auto wr_u32 = [&](std::size_t off, uint32_t v) {
      std::memcpy(p.data() + off, &v, 4);
    };
  wr_f32(4, pressure);
  wr_u32(8, ts);
  wr_f32(12, acc[0]);
  wr_f32(16, acc[1]);
  wr_f32(20, acc[2]);
  wr_f32(24, gyr[0]);
  wr_f32(28, gyr[1]);
  wr_f32(32, gyr[2]);
  wr_f32(36, mag[0]);
  wr_f32(40, mag[1]);
  wr_f32(44, mag[2]);
  wr_f32(48, eul[0]);
  wr_f32(52, eul[1]);
  wr_f32(56, eul[2]);
  wr_f32(60, quat[0]);
  wr_f32(64, quat[1]);
  wr_f32(68, quat[2]);
  wr_f32(72, quat[3]);
  return p;
}

// ---------------------------------------------------------------------------
// 手册 §6.3.1 原始帧示例（82 字节）
// ---------------------------------------------------------------------------
static const std::vector<uint8_t> MANUAL_FRAME = {
  0x5A, 0xA5, 0x4C, 0x00, 0x6C, 0x51,
  0x91, 0x00, 0xA0, 0x3B, 0x01, 0xA8, 0x02, 0x97, 0xBD, 0xBB, 0x04, 0x00, 0x9C, 0xA0, 0x65, 0x3E,
  0xA2, 0x26, 0x45, 0x3F, 0x5C, 0xE7, 0x30, 0x3F, 0xE2, 0xD4, 0x5A, 0xC2, 0xE5, 0x9D, 0xA0, 0xC1,
  0xEB, 0x23, 0xEE, 0xC2, 0x78, 0x77, 0x99, 0x41, 0xAB, 0xAA, 0xD1, 0xC1, 0xAB, 0x2A, 0x0A, 0xC2,
  0x8D, 0xE1, 0x42, 0x42, 0x8F, 0x1D, 0xA8, 0xC1, 0x1E, 0x0C, 0x36, 0xC2, 0xE6, 0xE5, 0x5A, 0x3F,
  0xC1, 0x94, 0x9E, 0x3E, 0xB8, 0xC0, 0x9E, 0xBE, 0xBE, 0xDF, 0x8D, 0xBE
};

// ---------------------------------------------------------------------------

TEST(CRC, EmptyInput)
{
  EXPECT_EQ(crc16(nullptr, 0), 0U);
}

TEST(CRC, ManualExample)
{
  // 手册示例：header(4B) + payload(76B) → 0x516C
  uint16_t crc = crc16(MANUAL_FRAME.data(), 4);
  crc = [&]() {
      // 续算 payload
      std::vector<uint8_t> full(MANUAL_FRAME.begin(), MANUAL_FRAME.begin() + 4);
      full.insert(full.end(), MANUAL_FRAME.begin() + 6, MANUAL_FRAME.end());
      return crc16(full.data(), full.size());
    }();
  EXPECT_EQ(crc, 0x516CU);
}

TEST(Decoder, ManualFrameDecode)
{
  std::vector<ImuSolPacket> pkts;
  HipnucDecoder dec([&](const ImuSolPacket & p) {pkts.push_back(p);});
  dec.feed(MANUAL_FRAME.data(), MANUAL_FRAME.size());

  ASSERT_EQ(pkts.size(), 1U);
  // acc(G): 0.2242  0.7701  0.6910
  EXPECT_NEAR(pkts[0].acc_x / G, 0.2242F, 1e-3F);
  EXPECT_NEAR(pkts[0].acc_y / G, 0.7701F, 1e-3F);
  EXPECT_NEAR(pkts[0].acc_z / G, 0.6910F, 1e-3F);
  EXPECT_EQ(pkts[0].timestamp_ms, 310205U);
}

TEST(Decoder, BasicImuSol)
{
  std::vector<ImuSolPacket> pkts;
  HipnucDecoder dec([&](const ImuSolPacket & p) {pkts.push_back(p);});

  auto payload = make_imusol(
    0, 25, 0.0F, 0,
    {0.0F, 0.0F, 1.0F}, {1.0F, 0.0F, 0.0F},
    {10.0F, 0.0F, 0.0F}, {0.0F, 0.0F, 0.0F},
    {1.0F, 0.0F, 0.0F, 0.0F});
  auto frame = build_frame(payload);
  dec.feed(frame.data(), frame.size());

  ASSERT_EQ(pkts.size(), 1U);
  EXPECT_NEAR(pkts[0].acc_z, G, 1e-3F);
  EXPECT_NEAR(pkts[0].gyro_x, 1.0F * D2R, 1e-5F);
  EXPECT_NEAR(pkts[0].mag_x, 10.0F, 1e-4F);
  EXPECT_NEAR(pkts[0].quat_w, 1.0F, 1e-5F);
}

TEST(Decoder, TemperatureAndPressure)
{
  std::vector<ImuSolPacket> pkts;
  HipnucDecoder dec([&](const ImuSolPacket & p) {pkts.push_back(p);});

  auto frame = build_frame(make_imusol(0, -20, 101325.0F));
  dec.feed(frame.data(), frame.size());

  ASSERT_EQ(pkts.size(), 1U);
  EXPECT_NEAR(pkts[0].temperature, -20.0F, 0.1F);
  EXPECT_NEAR(pkts[0].pressure, 101325.0F, 1.0F);
}

TEST(Decoder, Timestamp)
{
  std::vector<ImuSolPacket> pkts;
  HipnucDecoder dec([&](const ImuSolPacket & p) {pkts.push_back(p);});

  auto frame = build_frame(make_imusol(0, 25, 0.0F, 310205U));
  dec.feed(frame.data(), frame.size());

  ASSERT_EQ(pkts.size(), 1U);
  EXPECT_EQ(pkts[0].timestamp_ms, 310205U);
}

TEST(Decoder, DeviceId)
{
  std::vector<ImuSolPacket> pkts;
  HipnucDecoder dec([&](const ImuSolPacket & p) {pkts.push_back(p);});

  auto frame = build_frame(make_imusol(7));
  dec.feed(frame.data(), frame.size());

  ASSERT_EQ(pkts.size(), 1U);
  EXPECT_EQ(pkts[0].device_id, 7U);
}

TEST(Decoder, CrcErrorRejected)
{
  std::vector<ImuSolPacket> pkts;
  HipnucDecoder dec([&](const ImuSolPacket & p) {pkts.push_back(p);});

  auto frame = build_frame(make_imusol());
  frame[4] ^= 0xFF;  // 破坏 CRC 低字节
  dec.feed(frame.data(), frame.size());

  EXPECT_EQ(pkts.size(), 0U);
}

TEST(Decoder, MultipleFrames)
{
  std::vector<ImuSolPacket> pkts;
  HipnucDecoder dec([&](const ImuSolPacket & p) {pkts.push_back(p);});

  auto frame = build_frame(make_imusol());
  std::vector<uint8_t> stream;
  for (int i = 0; i < 5; ++i) {
    stream.insert(stream.end(), frame.begin(), frame.end());
  }
  dec.feed(stream.data(), stream.size());

  EXPECT_EQ(pkts.size(), 5U);
}

TEST(Decoder, FragmentedDelivery)
{
  std::vector<ImuSolPacket> pkts;
  HipnucDecoder dec([&](const ImuSolPacket & p) {pkts.push_back(p);});

  auto frame = build_frame(make_imusol({}, 25, 0.0F, 0, {0.0F, 0.0F, 1.0F}));
  for (auto b : frame) {
    dec.feed(&b, 1);
  }

  ASSERT_EQ(pkts.size(), 1U);
  EXPECT_NEAR(pkts[0].acc_z, G, 1e-3F);
}

TEST(Decoder, GarbageBeforeFrame)
{
  std::vector<ImuSolPacket> pkts;
  HipnucDecoder dec([&](const ImuSolPacket & p) {pkts.push_back(p);});

  std::vector<uint8_t> stream = {0x00, 0x5A, 0xFF, 0xA5, 0x00, 0xAB};
  auto frame = build_frame(make_imusol());
  stream.insert(stream.end(), frame.begin(), frame.end());
  dec.feed(stream.data(), stream.size());

  EXPECT_EQ(pkts.size(), 1U);
}

TEST(Decoder, TwoConsecutiveFrames)
{
  std::vector<ImuSolPacket> pkts;
  HipnucDecoder dec([&](const ImuSolPacket & p) {pkts.push_back(p);});

  auto f1 = build_frame(make_imusol(0, 25, 0.0F, 0, {1.0F, 0.0F, 0.0F}));
  auto f2 = build_frame(make_imusol(0, 25, 0.0F, 0, {0.0F, 1.0F, 0.0F}));
  f1.insert(f1.end(), f2.begin(), f2.end());
  dec.feed(f1.data(), f1.size());

  ASSERT_EQ(pkts.size(), 2U);
  EXPECT_NEAR(pkts[0].acc_x, G, 1e-3F);
  EXPECT_NEAR(pkts[1].acc_y, G, 1e-3F);
}
