#ifndef PAGER100_LORA_BRIDGE__POSIX_SERIAL_TRANSPORT_HPP_
#define PAGER100_LORA_BRIDGE__POSIX_SERIAL_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "pager100_lora_bridge/serial_transport.hpp"

namespace pager100_lora_bridge
{

class PosixSerialTransport final : public SerialTransport
{
public:
  PosixSerialTransport() = default;
  ~PosixSerialTransport() override;

  bool open(
    const std::string & port,
    int baudrate,
    std::string & error_message) override;
  void close() override;
  bool is_open() const override;
  int read(
    std::uint8_t * buffer,
    std::size_t max_length,
    std::string & error_message) override;
  bool write(
    const std::uint8_t * data,
    std::size_t length,
    std::string & error_message) override;

private:
  int fd_ {-1};
};

}  // namespace pager100_lora_bridge

#endif  // PAGER100_LORA_BRIDGE__POSIX_SERIAL_TRANSPORT_HPP_
