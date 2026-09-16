#ifndef SMIT_ROS_DRIVER__SMIT_PROTOCOL_HPP_
#define SMIT_ROS_DRIVER__SMIT_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace smit_ros_driver
{

/// 配置包（msg_id 0x02）解码结果
struct SmitConfigInfo
{
  std::array<uint8_t, 14> sn {};
  std::array<uint8_t, 3> hw_ver {};
  std::array<uint8_t, 3> fpga_ver {};
  std::array<uint8_t, 3> mcu_ver {};
  int motor_speed {0};       ///< 电机转速 Hz（10 / 15 / 20）
  int lidar_freq {0};        ///< 点频 points/s（10000 / 20000 / 40000）
  int instant_speed {0};     ///< 实时转速
  int lidar_temp {0};        ///< 温度 °C
  float lidar_volt {0.0F};   ///< 电压 V
  uint8_t lidar_state {0};   ///< 雷达状态
};

/// 单个测距点（角度已折算到 [0, 360) 度）
struct SmitPoint
{
  float angle_deg {0.0F};
  float distance_m {0.0F};
  float intensity {0.0F};
};

/// 点云包（msg_id 0x01）解码结果。
/// crc_ok=false 表示该包 CRC 校验失败、点内容无效，仅作为"坏包"信号上抛，
/// 供扫描装配器丢弃正在积累的半圈数据。
struct SmitPointPacket
{
  static constexpr std::size_t kMaxPoints = 64;

  std::array<SmitPoint, kMaxPoints> points {};
  std::size_t point_count {0};
  uint32_t timestamp {0};    ///< 包尾 24bit 原始时间戳
  bool crc_ok {false};
};

/// SMIT 串口雷达字节流解析器（ROS-free，可单测）。
///
/// 帧格式（长度域大端）：
///   [0xA5][0x5A][LEN_H][LEN_L][PAYLOAD(LEN bytes)]
/// PAYLOAD[0] 为 msg_id（0x01 点云 / 0x02 配置），PAYLOAD[LEN-1] 为 CRC8
/// （多项式 0x07，初值 0x00），覆盖 PAYLOAD[0..LEN-2]。
class SmitProtocol
{
public:
  using ConfigCallback = std::function<void(const SmitConfigInfo &)>;
  using PointCallback = std::function<void(const SmitPointPacket &)>;

  SmitProtocol(ConfigCallback config_callback, PointCallback point_callback);

  /// 喂入字节流；每解出一个完整配置包或点云包就回调一次。
  void feed(const uint8_t * data, std::size_t length);

  /// CRC8（多项式 0x07，初值 0x00）。
  static uint8_t crc8(const uint8_t * data, std::size_t length);

private:
  void process_frame(const uint8_t * frame, std::size_t length);
  void parse_config(const uint8_t * payload, std::size_t length);
  void parse_points(const uint8_t * payload, std::size_t length, bool crc_ok);

  ConfigCallback config_callback_;
  PointCallback point_callback_;
  std::vector<uint8_t> buffer_;
};

}  // namespace smit_ros_driver

#endif  // SMIT_ROS_DRIVER__SMIT_PROTOCOL_HPP_
