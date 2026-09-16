#ifndef CH020_IMU_DRIVER__SERIAL_TRANSPORT_HPP_
#define CH020_IMU_DRIVER__SERIAL_TRANSPORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace ch020_imu_driver
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
    uint8_t * buffer,
    std::size_t max_length,
    std::string & error_message) = 0;
};

}  // namespace ch020_imu_driver

#endif  // CH020_IMU_DRIVER__SERIAL_TRANSPORT_HPP_
