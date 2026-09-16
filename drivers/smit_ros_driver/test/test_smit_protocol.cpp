#include "smit_ros_driver/smit_protocol.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "smit_ros_driver/smit_lidar_driver.hpp"

using smit_ros_driver::SmitConfigInfo;
using smit_ros_driver::SmitPoint;
using smit_ros_driver::SmitPointPacket;
using smit_ros_driver::SmitProtocol;
using smit_ros_driver::SmitScan;
using smit_ros_driver::SmitScanAssembler;

// ---------------------------------------------------------------------------
// 测试辅助：按真实帧格式构造字节序列
//   [0xA5][0x5A][LEN_H][LEN_L][msg_id][body...][CRC8]
// ---------------------------------------------------------------------------

static std::vector<uint8_t> build_frame(uint8_t msg_id, const std::vector<uint8_t> & body)
{
  const uint16_t payload_len = static_cast<uint16_t>(body.size() + 2);
  std::vector<uint8_t> frame = {
    0xA5, 0x5A,
    static_cast<uint8_t>((payload_len >> 8) & 0xFFU),
    static_cast<uint8_t>(payload_len & 0xFFU),
    msg_id};
  frame.insert(frame.end(), body.begin(), body.end());
  frame.push_back(SmitProtocol::crc8(frame.data() + 4, frame.size() - 4));
  return frame;
}

static void append_u16_be(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
}

static std::vector<uint8_t> make_config_body(
  uint8_t motor_code = 3,    // 3 → 20Hz
  uint8_t freq_code = 1,     // 1 → 20000 points/s
  uint16_t instant_speed = 600,
  uint8_t temp = 42,
  uint8_t volt_raw = 121,    // 12.1V
  uint8_t state = 1)
{
  std::vector<uint8_t> body;
  const char sn[] = "SMIT1234567890";   // 14 字节
  body.insert(body.end(), sn, sn + 14);
  body.insert(body.end(), {1, 2, 3});    // hw_ver
  body.insert(body.end(), {4, 5, 6});    // fpga_ver
  body.insert(body.end(), {7, 8, 9});    // mcu_ver
  body.push_back(0);                     // 保留字节
  body.push_back(motor_code);
  append_u16_be(body, instant_speed);
  body.push_back(freq_code);
  body.push_back(temp);
  body.push_back(volt_raw);
  body.push_back(state);
  return body;
}

static std::vector<uint8_t> make_point_body(
  float start_angle_deg,
  float delta_angle_deg,
  const std::vector<std::pair<uint8_t, uint16_t>> & points,  // (强度, 距离原始值)
  uint32_t timestamp = 0)
{
  std::vector<uint8_t> body;
  append_u16_be(body, static_cast<uint16_t>(std::lround(start_angle_deg * 100.0F)));
  append_u16_be(body, static_cast<uint16_t>(std::lround(delta_angle_deg * 10000.0F)));
  for (const auto & point : points) {
    body.push_back(point.first);
    append_u16_be(body, point.second);
  }
  body.push_back(static_cast<uint8_t>((timestamp >> 16) & 0xFFU));
  body.push_back(static_cast<uint8_t>((timestamp >> 8) & 0xFFU));
  body.push_back(static_cast<uint8_t>(timestamp & 0xFFU));
  return body;
}

static SmitPointPacket make_packet(
  float start_angle_deg,
  std::size_t point_count,
  float delta_angle_deg = 0.18F,
  bool crc_ok = true)
{
  SmitPointPacket packet;
  packet.crc_ok = crc_ok;
  if (!crc_ok) {
    return packet;
  }
  packet.point_count = point_count;
  for (std::size_t i = 0; i < point_count; ++i) {
    packet.points[i].angle_deg = start_angle_deg + delta_angle_deg * static_cast<float>(i);
    packet.points[i].distance_m = 1.0F;
    packet.points[i].intensity = 10.0F;
  }
  return packet;
}

// ---------------------------------------------------------------------------
// CRC8
// ---------------------------------------------------------------------------

TEST(SmitProtocol, Crc8MatchesKnownVector)
{
  // CRC-8/SMBUS（poly 0x07，初值 0）标准校验向量："123456789" → 0xF4
  const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(SmitProtocol::crc8(data, sizeof(data)), 0xF4U);
}

// ---------------------------------------------------------------------------
// 配置包解码
// ---------------------------------------------------------------------------

TEST(SmitProtocol, DecodesConfigPacket)
{
  std::vector<SmitConfigInfo> configs;
  SmitProtocol protocol(
    [&configs](const SmitConfigInfo & config) {configs.push_back(config);},
    [](const SmitPointPacket &) {});

  const auto frame = build_frame(0x02, make_config_body());
  protocol.feed(frame.data(), frame.size());

  ASSERT_EQ(configs.size(), 1U);
  const SmitConfigInfo & config = configs.front();
  EXPECT_EQ(std::string(config.sn.begin(), config.sn.end()), "SMIT1234567890");
  EXPECT_EQ(config.hw_ver[0], 1U);
  EXPECT_EQ(config.fpga_ver[1], 5U);
  EXPECT_EQ(config.mcu_ver[2], 9U);
  EXPECT_EQ(config.motor_speed, 20);
  EXPECT_EQ(config.instant_speed, 600);
  EXPECT_EQ(config.lidar_freq, 20000);
  EXPECT_EQ(config.lidar_temp, 42);
  EXPECT_FLOAT_EQ(config.lidar_volt, 12.1F);
  EXPECT_EQ(config.lidar_state, 1U);
}

TEST(SmitProtocol, MapsMotorAndFrequencyCodes)
{
  std::vector<SmitConfigInfo> configs;
  SmitProtocol protocol(
    [&configs](const SmitConfigInfo & config) {configs.push_back(config);},
    [](const SmitPointPacket &) {});

  const auto frame = build_frame(0x02, make_config_body(2, 0));
  protocol.feed(frame.data(), frame.size());

  ASSERT_EQ(configs.size(), 1U);
  EXPECT_EQ(configs.front().motor_speed, 15);
  EXPECT_EQ(configs.front().lidar_freq, 10000);
}

TEST(SmitProtocol, DropsConfigPacketWithBadCrc)
{
  std::vector<SmitConfigInfo> configs;
  SmitProtocol protocol(
    [&configs](const SmitConfigInfo & config) {configs.push_back(config);},
    [](const SmitPointPacket &) {});

  auto frame = build_frame(0x02, make_config_body());
  frame.back() ^= 0xFFU;
  protocol.feed(frame.data(), frame.size());

  EXPECT_TRUE(configs.empty());
}

// ---------------------------------------------------------------------------
// 点云包解码
// ---------------------------------------------------------------------------

TEST(SmitProtocol, DecodesPointPacket)
{
  std::vector<SmitPointPacket> packets;
  SmitProtocol protocol(
    [](const SmitConfigInfo &) {},
    [&packets](const SmitPointPacket & packet) {packets.push_back(packet);});

  const auto body = make_point_body(
    100.0F, 0.18F, {{10, 400}, {20, 800}, {30, 1200}}, 0x010203U);
  const auto frame = build_frame(0x01, body);
  protocol.feed(frame.data(), frame.size());

  ASSERT_EQ(packets.size(), 1U);
  const SmitPointPacket & packet = packets.front();
  EXPECT_TRUE(packet.crc_ok);
  ASSERT_EQ(packet.point_count, 3U);
  EXPECT_NEAR(packet.points[0].angle_deg, 100.0F, 1e-3F);
  EXPECT_NEAR(packet.points[1].angle_deg, 100.18F, 1e-3F);
  EXPECT_NEAR(packet.points[2].angle_deg, 100.36F, 1e-3F);
  EXPECT_FLOAT_EQ(packet.points[0].distance_m, 1.0F);    // 400 * 0.0025
  EXPECT_FLOAT_EQ(packet.points[1].distance_m, 2.0F);    // 800 * 0.0025
  EXPECT_FLOAT_EQ(packet.points[2].distance_m, 3.0F);    // 1200 * 0.0025
  EXPECT_FLOAT_EQ(packet.points[0].intensity, 10.0F);
  EXPECT_EQ(packet.timestamp, 0x010203U);
}

TEST(SmitProtocol, WrapsAngleAbove360)
{
  std::vector<SmitPointPacket> packets;
  SmitProtocol protocol(
    [](const SmitConfigInfo &) {},
    [&packets](const SmitPointPacket & packet) {packets.push_back(packet);});

  const auto body = make_point_body(359.9F, 0.18F, {{1, 100}, {2, 100}});
  const auto frame = build_frame(0x01, body);
  protocol.feed(frame.data(), frame.size());

  ASSERT_EQ(packets.size(), 1U);
  ASSERT_EQ(packets.front().point_count, 2U);
  EXPECT_NEAR(packets.front().points[0].angle_deg, 359.9F, 1e-3F);
  EXPECT_NEAR(packets.front().points[1].angle_deg, 0.08F, 1e-3F);
}

TEST(SmitProtocol, DropsPointPacketWhenDeltaAngleOutOfRange)
{
  std::vector<SmitPointPacket> packets;
  SmitProtocol protocol(
    [](const SmitConfigInfo &) {},
    [&packets](const SmitPointPacket & packet) {packets.push_back(packet);});

  for (const float delta : {0.17F, 0.19F}) {
    const auto frame = build_frame(0x01, make_point_body(0.0F, delta, {{1, 100}}));
    protocol.feed(frame.data(), frame.size());
  }
  EXPECT_TRUE(packets.empty());

  const auto frame = build_frame(0x01, make_point_body(0.0F, 0.18F, {{1, 100}}));
  protocol.feed(frame.data(), frame.size());
  EXPECT_EQ(packets.size(), 1U);
}

TEST(SmitProtocol, DropsPointPacketWithTooManyPoints)
{
  std::vector<SmitPointPacket> packets;
  SmitProtocol protocol(
    [](const SmitConfigInfo &) {},
    [&packets](const SmitPointPacket & packet) {packets.push_back(packet);});

  const std::vector<std::pair<uint8_t, uint16_t>> points(65, {1, 100});
  const auto frame = build_frame(0x01, make_point_body(0.0F, 0.18F, points));
  protocol.feed(frame.data(), frame.size());
  EXPECT_TRUE(packets.empty());
}

TEST(SmitProtocol, ReportsCrcFailureForPointPacket)
{
  std::vector<SmitPointPacket> packets;
  SmitProtocol protocol(
    [](const SmitConfigInfo &) {},
    [&packets](const SmitPointPacket & packet) {packets.push_back(packet);});

  auto frame = build_frame(0x01, make_point_body(100.0F, 0.18F, {{1, 100}}));
  frame.back() ^= 0xFFU;
  protocol.feed(frame.data(), frame.size());

  ASSERT_EQ(packets.size(), 1U);
  EXPECT_FALSE(packets.front().crc_ok);
  EXPECT_EQ(packets.front().point_count, 0U);
}

TEST(SmitProtocol, IgnoresUnknownMsgId)
{
  int callback_count = 0;
  SmitProtocol protocol(
    [&callback_count](const SmitConfigInfo &) {++callback_count;},
    [&callback_count](const SmitPointPacket &) {++callback_count;});

  const auto frame = build_frame(0x04, {0x00});
  protocol.feed(frame.data(), frame.size());
  EXPECT_EQ(callback_count, 0);
}

// ---------------------------------------------------------------------------
// 帧切分：粘包 / 半包 / 垃圾字节重同步
// ---------------------------------------------------------------------------

TEST(SmitProtocol, HandlesStickyPackets)
{
  std::vector<SmitConfigInfo> configs;
  std::vector<SmitPointPacket> packets;
  SmitProtocol protocol(
    [&configs](const SmitConfigInfo & config) {configs.push_back(config);},
    [&packets](const SmitPointPacket & packet) {packets.push_back(packet);});

  std::vector<uint8_t> stream = build_frame(0x02, make_config_body());
  const auto point_frame = build_frame(0x01, make_point_body(10.0F, 0.18F, {{1, 100}}));
  stream.insert(stream.end(), point_frame.begin(), point_frame.end());

  protocol.feed(stream.data(), stream.size());
  EXPECT_EQ(configs.size(), 1U);
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_TRUE(packets.front().crc_ok);
}

TEST(SmitProtocol, HandlesSplitPacket)
{
  std::vector<SmitPointPacket> packets;
  SmitProtocol protocol(
    [](const SmitConfigInfo &) {},
    [&packets](const SmitPointPacket & packet) {packets.push_back(packet);});

  const auto frame = build_frame(0x01, make_point_body(10.0F, 0.18F, {{1, 100}}));

  // 逐字节喂入，模拟串口任意位置拆包
  for (const uint8_t byte : frame) {
    protocol.feed(&byte, 1);
  }
  ASSERT_EQ(packets.size(), 1U);
  EXPECT_NEAR(packets.front().points[0].angle_deg, 10.0F, 1e-3F);
}

TEST(SmitProtocol, ResyncsAfterGarbageBytes)
{
  std::vector<SmitPointPacket> packets;
  SmitProtocol protocol(
    [](const SmitConfigInfo &) {},
    [&packets](const SmitPointPacket & packet) {packets.push_back(packet);});

  const std::vector<uint8_t> garbage = {
    0x00, 0xA5, 0x11,                    // 半个假包头
    0xA5, 0x5A, 0xFF, 0xFF,              // 异常长度（>2048），丢掉重同步
    0xA5};                               // 孤立 0xA5
  const auto frame = build_frame(0x01, make_point_body(10.0F, 0.18F, {{1, 100}}));

  std::vector<uint8_t> stream = garbage;
  stream.insert(stream.end(), frame.begin(), frame.end());
  protocol.feed(stream.data(), stream.size());

  ASSERT_EQ(packets.size(), 1U);
  EXPECT_TRUE(packets.front().crc_ok);
}

// ---------------------------------------------------------------------------
// SmitScanAssembler：一圈点装配
// ---------------------------------------------------------------------------

TEST(SmitScanAssembler, NoOutputBeforeConfig)
{
  std::vector<SmitScan> scans;
  SmitScanAssembler assembler(
    -180.0, 180.0, [&scans](const SmitScan & scan) {scans.push_back(scan);});

  assembler.add_packet(make_packet(100.0F, 4));
  assembler.add_packet(make_packet(190.0F, 4));
  assembler.add_packet(make_packet(100.0F, 4));
  assembler.add_packet(make_packet(190.0F, 4));
  EXPECT_TRUE(scans.empty());
}

TEST(SmitScanAssembler, FullCircleEmitsScanOnBoundaryCross)
{
  std::vector<SmitScan> scans;
  SmitScanAssembler assembler(
    -180.0, 180.0, [&scans](const SmitScan & scan) {scans.push_back(scan);});
  assembler.set_config(10, 10000);

  assembler.add_packet(make_packet(100.0F, 4));   // 尚未跨边界，开始前的点被丢弃
  assembler.add_packet(make_packet(190.0F, 4));   // 跨过 180°，开始积累
  assembler.add_packet(make_packet(300.0F, 4));
  assembler.add_packet(make_packet(100.0F, 4));
  assembler.add_packet(make_packet(190.0F, 4));   // 再次跨过 180°，完成一圈

  ASSERT_EQ(scans.size(), 1U);
  const SmitScan & scan = scans.front();
  EXPECT_EQ(scan.points.size(), 12U);
  EXPECT_NEAR(scan.points.front().angle_deg, 190.0, 1e-3);
  EXPECT_EQ(scan.motor_speed, 10);
  EXPECT_EQ(scan.lidar_freq, 10000);
  EXPECT_DOUBLE_EQ(scan.angle_min_deg, -180.0);
  EXPECT_DOUBLE_EQ(scan.angle_max_deg, 180.0);
}

TEST(SmitScanAssembler, WindowEmitsScanWhenLeavingWindow)
{
  std::vector<SmitScan> scans;
  SmitScanAssembler assembler(
    10.0, 80.0, [&scans](const SmitScan & scan) {scans.push_back(scan);});
  assembler.set_config(10, 10000);

  assembler.add_packet(make_packet(0.0F, 2));     // 窗外
  assembler.add_packet(make_packet(20.0F, 2));    // 进入窗口，开始积累
  assembler.add_packet(make_packet(40.0F, 2));
  assembler.add_packet(make_packet(60.0F, 2));
  assembler.add_packet(make_packet(90.0F, 2));    // 离开窗口，上抛

  ASSERT_EQ(scans.size(), 1U);
  EXPECT_EQ(scans.front().points.size(), 6U);
  EXPECT_NEAR(scans.front().points.front().angle_deg, 20.0, 1e-3);
  EXPECT_NEAR(scans.front().points.back().angle_deg, 60.18, 1e-3);
}

TEST(SmitScanAssembler, CrcFailureDropsPartialScan)
{
  std::vector<SmitScan> scans;
  SmitScanAssembler assembler(
    10.0, 80.0, [&scans](const SmitScan & scan) {scans.push_back(scan);});
  assembler.set_config(10, 10000);

  assembler.add_packet(make_packet(0.0F, 2));
  assembler.add_packet(make_packet(20.0F, 2));    // 进入窗口
  assembler.add_packet(make_packet(40.0F, 2));
  assembler.add_packet(make_packet(0.0F, 0, 0.18F, false));   // 坏包，丢弃半圈
  assembler.add_packet(make_packet(90.0F, 2));    // 已不在积累状态，不应上抛

  EXPECT_TRUE(scans.empty());
}
