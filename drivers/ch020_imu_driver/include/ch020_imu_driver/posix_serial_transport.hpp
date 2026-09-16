#ifndef CH020_IMU_DRIVER__POSIX_SERIAL_TRANSPORT_HPP_
#define CH020_IMU_DRIVER__POSIX_SERIAL_TRANSPORT_HPP_

#include <string>

#include "ch020_imu_driver/serial_transport.hpp"

namespace ch020_imu_driver
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

}  // namespace ch020_imu_driver

#endif  // CH020_IMU_DRIVER__POSIX_SERIAL_TRANSPORT_HPP_
