#ifndef PAGER100_LORA_BRIDGE__SERIAL_FRAME_HPP_
#define PAGER100_LORA_BRIDGE__SERIAL_FRAME_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace pager100_lora_bridge
{

inline constexpr std::uint8_t kFrameSof = 0x7E;
inline constexpr std::uint8_t kFrameEsc = 0x7D;
inline constexpr std::uint8_t kFrameXor = 0x20;
inline constexpr std::uint8_t kFrameTypeData = 0x01;
inline constexpr std::size_t kMaxPayloadSize = 64;
inline constexpr std::uint16_t kCrc16Init = 0xFFFF;
inline constexpr std::uint16_t kCrc16Poly = 0xA001;

struct SerialFrame
{
  std::uint8_t frame_type {0};
  std::uint8_t sequence {0};
  std::vector<std::uint8_t> payload;
};

/// Modbus CRC16（低字节先行），与 TWS/PG100 手册一致。
std::uint16_t crc16(const std::uint8_t * data, std::size_t length);

/// 编码一帧；payload 超过 kMaxPayloadSize 时抛 std::invalid_argument。
std::vector<std::uint8_t> encode_frame(
  const std::vector<std::uint8_t> & payload,
  std::uint8_t sequence = 0,
  std::uint8_t frame_type = kFrameTypeData);

class SerialFrameDecoder
{
public:
  std::vector<SerialFrame> feed(const std::uint8_t * data, std::size_t length);
  std::vector<SerialFrame> feed(const std::vector<std::uint8_t> & data);
  std::optional<SerialFrame> feed_byte(std::uint8_t value);

private:
  std::optional<SerialFrame> finish_frame();

  std::vector<std::uint8_t> buffer_;
  bool in_frame_ {false};
  bool escape_ {false};
};

}  // namespace pager100_lora_bridge

#endif  // PAGER100_LORA_BRIDGE__SERIAL_FRAME_HPP_
