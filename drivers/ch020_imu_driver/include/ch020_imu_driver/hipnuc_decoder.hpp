#ifndef CH020_IMU_DRIVER__HIPNUC_DECODER_HPP_
#define CH020_IMU_DRIVER__HIPNUC_DECODER_HPP_

#include <cstdint>
#include <functional>
#include <vector>

namespace ch020_imu_driver
{

/// 0x91 IMUSOL 数据包（手册 §6，76 字节，已换算为 SI 单位）
struct ImuSolPacket
{
  uint8_t device_id;
  float temperature;    ///< °C
  float pressure;       ///< Pa
  uint32_t timestamp_ms;   ///< 上电后累计 ms
  float acc_x, acc_y, acc_z;     ///< m/s²
  float gyro_x, gyro_y, gyro_z;  ///< rad/s
  float mag_x, mag_y, mag_z;     ///< µT
  float roll, pitch, yaw;        ///< deg（ZYX 顺规）
  float quat_w, quat_x, quat_y, quat_z;
};

/// 字节流状态机，从串口连续字节流中解析 HiPNUC CH0X0 帧。
///
/// 帧格式（手册 Rev 1.1 §6，小端序）：
///   [0x5A][0xA5][LEN_L][LEN_H][CRC_L][CRC_H][PAYLOAD(LEN bytes)]
///
/// CRC-16/CCITT（多项式 0x1021，初值 0）覆盖：
///   [0x5A, 0xA5, LEN_L, LEN_H] + PAYLOAD
class HipnucDecoder
{
public:
  using PacketCallback = std::function<void(const ImuSolPacket &)>;

  explicit HipnucDecoder(PacketCallback callback);

  /// 喂入字节流，每解出一帧就回调一次。
  void feed(const uint8_t * data, std::size_t length);

private:
  enum class State : uint8_t
  {
    SOF1, SOF2, LEN1, LEN2, CRC1, CRC2, DATA
  };

  /// CRC-16/CCITT 增量计算（可多次调用以连接多段数据）。
  static uint16_t crc16_update(uint16_t crc, const uint8_t * data, std::size_t length);

  void process_frame();

  PacketCallback callback_;
  State state_ {State::SOF1};
  uint16_t payload_len_ {0};
  uint16_t crc_lo_ {0};
  uint16_t crc_rx_ {0};
  std::vector<uint8_t> buf_;
};

}  // namespace ch020_imu_driver

#endif  // CH020_IMU_DRIVER__HIPNUC_DECODER_HPP_
