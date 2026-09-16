#include "pager100_lora_bridge/serial_frame.hpp"

#include <stdexcept>

namespace pager100_lora_bridge
{

std::uint16_t crc16(const std::uint8_t * data, std::size_t length)
{
  std::uint16_t crc = kCrc16Init;
  for (std::size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x0001U) != 0U) {
        crc = static_cast<std::uint16_t>((crc >> 1) ^ kCrc16Poly);
      } else {
        crc = static_cast<std::uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

namespace
{

void append_escaped(std::vector<std::uint8_t> & output, std::uint8_t value)
{
  if (value == kFrameSof || value == kFrameEsc) {
    output.push_back(kFrameEsc);
    output.push_back(static_cast<std::uint8_t>(value ^ kFrameXor));
  } else {
    output.push_back(value);
  }
}

}  // namespace

std::vector<std::uint8_t> encode_frame(
  const std::vector<std::uint8_t> & payload,
  std::uint8_t sequence,
  std::uint8_t frame_type)
{
  if (payload.size() > kMaxPayloadSize) {
    throw std::invalid_argument("payload too large: " + std::to_string(payload.size()));
  }

  std::vector<std::uint8_t> body {frame_type, sequence, static_cast<std::uint8_t>(payload.size())};
  body.insert(body.end(), payload.begin(), payload.end());
  const std::uint16_t checksum = crc16(body.data(), body.size());
  body.push_back(static_cast<std::uint8_t>(checksum & 0xFFU));
  body.push_back(static_cast<std::uint8_t>((checksum >> 8) & 0xFFU));

  std::vector<std::uint8_t> frame {kFrameSof};
  for (const std::uint8_t value : body) {
    append_escaped(frame, value);
  }
  frame.push_back(kFrameSof);
  return frame;
}

std::vector<SerialFrame> SerialFrameDecoder::feed(const std::uint8_t * data, std::size_t length)
{
  std::vector<SerialFrame> frames;
  for (std::size_t index = 0; index < length; ++index) {
    if (auto frame = feed_byte(data[index])) {
      frames.push_back(std::move(*frame));
    }
  }
  return frames;
}

std::vector<SerialFrame> SerialFrameDecoder::feed(const std::vector<std::uint8_t> & data)
{
  return feed(data.data(), data.size());
}

std::optional<SerialFrame> SerialFrameDecoder::feed_byte(std::uint8_t value)
{
  if (value == kFrameSof) {
    auto frame = (in_frame_ && !buffer_.empty()) ? finish_frame() : std::nullopt;
    buffer_.clear();
    in_frame_ = true;
    escape_ = false;
    return frame;
  }

  if (!in_frame_) {
    return std::nullopt;
  }

  if (escape_) {
    buffer_.push_back(static_cast<std::uint8_t>(value ^ kFrameXor));
    escape_ = false;
    return std::nullopt;
  }

  if (value == kFrameEsc) {
    escape_ = true;
    return std::nullopt;
  }

  buffer_.push_back(value);
  if (buffer_.size() > kMaxPayloadSize + 5) {
    // 噪声流不发 SOF 时防止缓冲无限增长；当前帧直接作废。
    buffer_.clear();
    in_frame_ = false;
    escape_ = false;
  }
  return std::nullopt;
}

std::optional<SerialFrame> SerialFrameDecoder::finish_frame()
{
  if (buffer_.size() < 5) {
    return std::nullopt;
  }

  const std::uint8_t frame_type = buffer_[0];
  const std::uint8_t sequence = buffer_[1];
  const std::size_t length = buffer_[2];
  const std::size_t expected_length = 3 + length + 2;
  if (buffer_.size() != expected_length || length > kMaxPayloadSize) {
    return std::nullopt;
  }

  const std::uint16_t received_crc = static_cast<std::uint16_t>(
    buffer_[3 + length] | (buffer_[4 + length] << 8));
  if (crc16(buffer_.data(), 3 + length) != received_crc) {
    return std::nullopt;
  }

  return SerialFrame {
    frame_type,
    sequence,
    std::vector<std::uint8_t>(buffer_.begin() + 3, buffer_.begin() + 3 + length)};
}

}  // namespace pager100_lora_bridge
