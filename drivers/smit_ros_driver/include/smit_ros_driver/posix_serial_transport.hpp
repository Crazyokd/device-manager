#ifndef SMIT_ROS_DRIVER__POSIX_SERIAL_TRANSPORT_HPP_
#define SMIT_ROS_DRIVER__POSIX_SERIAL_TRANSPORT_HPP_

#include <string>

#include "smit_ros_driver/serial_transport.hpp"

namespace smit_ros_driver
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
    uint8_t * buffer,
    std::size_t max_length,
    std::string & error_message) override;

private:
  int fd_ {-1};
};

}  // namespace smit_ros_driver

#endif  // SMIT_ROS_DRIVER__POSIX_SERIAL_TRANSPORT_HPP_
