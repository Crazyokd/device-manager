#include "ch020_imu_driver/posix_serial_transport.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <sstream>

namespace ch020_imu_driver
{

namespace
{

std::string errno_to_string(const std::string & prefix)
{
  std::ostringstream stream;
  stream << prefix << ": " << std::strerror(errno);
  return stream.str();
}

speed_t baudrate_to_speed(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    case 115200:
    default:
      return B115200;
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

  const speed_t speed = baudrate_to_speed(baudrate);
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

}  // namespace ch020_imu_driver
