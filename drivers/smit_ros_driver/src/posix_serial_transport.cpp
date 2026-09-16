#include "smit_ros_driver/posix_serial_transport.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <sstream>

namespace smit_ros_driver
{

namespace
{

std::string errno_to_string(const std::string & prefix)
{
  std::ostringstream stream;
  stream << prefix << ": " << std::strerror(errno);
  return stream.str();
}

/// SMIT 雷达固件只支持 9600 / 115200 / 921600 / 2000000，其余波特率直接拒绝。
bool baudrate_to_speed(int baudrate, speed_t & speed)
{
  switch (baudrate) {
    case 9600:
      speed = B9600;
      return true;
    case 115200:
      speed = B115200;
      return true;
    case 921600:
      speed = B921600;
      return true;
    case 2000000:
      speed = B2000000;
      return true;
    default:
      return false;
  }
}

}  // namespace

PosixSerialTransport::~PosixSerialTransport()
{
  close();
}

bool PosixSerialTransport::open(
  const std::string & port,
  int baudrate,
  std::string & error_message)
{
  close();

  speed_t speed {};
  if (!baudrate_to_speed(baudrate, speed)) {
    std::ostringstream stream;
    stream << "不支持的波特率 " << baudrate << "（仅支持 9600/115200/921600/2000000）";
    error_message = stream.str();
    return false;
  }

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    error_message = errno_to_string("无法打开串口 " + port);
    return false;
  }

  struct termios tio {};
  if (::tcgetattr(fd_, &tio) < 0) {
    error_message = errno_to_string("读取串口属性失败 " + port);
    close();
    return false;
  }
  ::cfmakeraw(&tio);

  ::cfsetispeed(&tio, speed);
  ::cfsetospeed(&tio, speed);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 10;  // 1s 超时，使 read() 可被停止与断线检测打断
  if (::tcsetattr(fd_, TCSANOW, &tio) < 0) {
    error_message = errno_to_string("设置串口属性失败 " + port);
    close();
    return false;
  }

  error_message.clear();
  return true;
}

void PosixSerialTransport::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool PosixSerialTransport::is_open() const
{
  return fd_ >= 0;
}

int PosixSerialTransport::read(
  uint8_t * buffer,
  std::size_t max_length,
  std::string & error_message)
{
  if (!is_open()) {
    error_message = "串口尚未打开";
    return -1;
  }

  const ssize_t received = ::read(fd_, buffer, max_length);
  if (received < 0) {
    error_message = errno_to_string("读取串口失败");
    return -1;
  }

  error_message.clear();
  return static_cast<int>(received);
}

}  // namespace smit_ros_driver
