#ifndef PAGER100_LORA_BRIDGE__SERIAL_TRANSPORT_HPP_
#define PAGER100_LORA_BRIDGE__SERIAL_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace pager100_lora_bridge
{

class SerialTransport
{
public:
  virtual ~SerialTransport() = default;

  virtual bool open(
    const std::string & port,
    int baudrate,
    std::string & error_message) = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;

  /// 返回读取的字节数；0 表示超时无数据；负值表示读错误（error_message 已填写）。
  virtual int read(
    std::uint8_t * buffer,
    std::size_t max_length,
    std::string & error_message) = 0;

  /// 完整写入一段字节；失败返回 false（error_message 已填写）。
  virtual bool write(
    const std::uint8_t * data,
    std::size_t length,
    std::string & error_message) = 0;
};

}  // namespace pager100_lora_bridge

#endif  // PAGER100_LORA_BRIDGE__SERIAL_TRANSPORT_HPP_
