#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "pager100_lora_bridge/serial_frame.hpp"

namespace pager100_lora_bridge
{
namespace
{

SerialFrame decode_one(const std::vector<std::uint8_t> & frame)
{
  const auto decoded = SerialFrameDecoder().feed(frame);
  EXPECT_EQ(decoded.size(), 1U);
  return decoded.front();
}

TEST(SerialFrame, RoundTripPayload)
{
  const std::vector<std::uint8_t> payload {1, 2, 3, 4};
  const auto frame = decode_one(encode_frame(payload, 7));
  EXPECT_EQ(frame.sequence, 7);
  EXPECT_EQ(frame.payload, payload);
}

TEST(SerialFrame, EscapesFrameControlBytes)
{
  const std::vector<std::uint8_t> payload {kFrameSof, kFrameEsc, 0x11};
  const auto encoded = encode_frame(payload, 1);
  for (std::size_t index = 1; index + 1 < encoded.size(); ++index) {
    EXPECT_NE(encoded[index], kFrameSof) << "控制字节未转义，偏移 " << index;
  }
  EXPECT_EQ(decode_one(encoded).payload, payload);
}

TEST(SerialFrame, RejectsCrcError)
{
  auto encoded = encode_frame({'a', 'b', 'c'}, 2);
  encoded[encoded.size() - 3] ^= 0x01;
  EXPECT_TRUE(SerialFrameDecoder().feed(encoded).empty());
}

TEST(SerialFrame, RejectsOversizedPayload)
{
  EXPECT_THROW(encode_frame(std::vector<std::uint8_t>(kMaxPayloadSize + 1)), std::invalid_argument);
}

TEST(SerialFrame, DecodesByteByByte)
{
  const std::vector<std::uint8_t> payload {9, 8, 7};
  const auto encoded = encode_frame(payload, 3);
  SerialFrameDecoder decoder;
  std::vector<SerialFrame> frames;
  for (const std::uint8_t value : encoded) {
    if (auto frame = decoder.feed_byte(value)) {
      frames.push_back(*frame);
    }
  }
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames.front().payload, payload);
}

TEST(SerialFrame, SkipsGarbageBeforeSof)
{
  const std::vector<std::uint8_t> payload {0x42};
  auto stream = std::vector<std::uint8_t> {0x00, 0x11, 0x22};
  const auto encoded = encode_frame(payload, 4);
  stream.insert(stream.end(), encoded.begin(), encoded.end());
  const auto frames = SerialFrameDecoder().feed(stream);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames.front().payload, payload);
}

TEST(SerialFrame, DecodesBackToBackFrames)
{
  auto stream = encode_frame({1}, 1);
  const auto second = encode_frame({2}, 2);
  stream.insert(stream.end(), second.begin(), second.end());
  const auto frames = SerialFrameDecoder().feed(stream);
  ASSERT_EQ(frames.size(), 2U);
  EXPECT_EQ(frames[0].payload, std::vector<std::uint8_t>({1}));
  EXPECT_EQ(frames[1].payload, std::vector<std::uint8_t>({2}));
}

}  // namespace
}  // namespace pager100_lora_bridge
