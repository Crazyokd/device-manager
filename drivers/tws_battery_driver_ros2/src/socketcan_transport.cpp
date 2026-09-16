#include "tws_battery_driver_ros2/socketcan_transport.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstring>
#include <sstream>

#include "tws_battery_driver_ros2/modbus_response_assembler.hpp"

namespace tws_battery_driver_ros2
{

namespace
{

constexpr uint8_t kReadHoldingRegistersFunctionCode = 0x03U;

std::string errno_to_string(const std::string & prefix)
{
  std::ostringstream stream;
  stream << prefix << ": " << std::strerror(errno);
  return stream.str();
}

}  // namespace

SocketCanTransport::SocketCanTransport(
  const std::string & interface_name,
  uint32_t can_id,
  bool use_extended_frame,
  int timeout_ms)
: interface_name_(interface_name),
  can_id_(use_extended_frame ? (can_id & CAN_EFF_MASK) : (can_id & CAN_SFF_MASK)),
  use_extended_frame_(use_extended_frame),
  timeout_ms_(timeout_ms),
  socket_fd_(-1)
{
}

SocketCanTransport::~SocketCanTransport()
{
  disconnect();
}

bool SocketCanTransport::connect(std::string & error_message)
{
  std::lock_guard<std::mutex> lock(io_mutex_);

  disconnect();

  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    error_message = errno_to_string("创建 CAN socket 失败");
    return false;
  }

  const int recv_own_messages = 0;
  const int loopback = 0;
  (void)setsockopt(
    socket_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
    &recv_own_messages, sizeof(recv_own_messages));
  (void)setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

  const can_filter filter {
    static_cast<canid_t>(can_id_ | (use_extended_frame_ ? CAN_EFF_FLAG : 0U)),
    static_cast<canid_t>(
      (use_extended_frame_ ? CAN_EFF_MASK : CAN_SFF_MASK) |
      (use_extended_frame_ ? CAN_EFF_FLAG : 0U))
  };
  if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
    error_message = errno_to_string("设置 CAN 过滤器失败");
    disconnect();
    return false;
  }

  const int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    error_message = errno_to_string("设置 CAN socket 非阻塞失败");
    disconnect();
    return false;
  }

  ifreq if_request {};
  std::strncpy(if_request.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if_request.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(socket_fd_, SIOCGIFINDEX, &if_request) < 0) {
    error_message = errno_to_string("读取 CAN 接口索引失败");
    disconnect();
    return false;
  }

  sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = if_request.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    error_message = errno_to_string("绑定 CAN 接口失败");
    disconnect();
    return false;
  }

  error_message.clear();
  return true;
}

void SocketCanTransport::disconnect()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool SocketCanTransport::is_connected() const
{
  return socket_fd_ >= 0;
}

bool SocketCanTransport::read_holding_registers(
  uint8_t device_address,
  uint16_t register_address,
  uint16_t register_count,
  std::vector<uint8_t> & register_bytes,
  std::string & error_message)
{
  if (register_count == 0U) {
    error_message = "寄存器数量不能为 0";
    return false;
  }

  std::vector<uint8_t> request_bytes {
    device_address,
    kReadHoldingRegistersFunctionCode,
    static_cast<uint8_t>((register_address >> 8) & 0xFFU),
    static_cast<uint8_t>(register_address & 0xFFU),
    static_cast<uint8_t>((register_count >> 8) & 0xFFU),
    static_cast<uint8_t>(register_count & 0xFFU)
  };
  append_crc(request_bytes);

  std::vector<uint8_t> response_bytes;
  if (!exchange_frame(request_bytes, response_bytes, error_message)) {
    return false;
  }

  if (!validate_crc(response_bytes)) {
    error_message = "BMS 响应 CRC 校验失败";
    return false;
  }

  const uint8_t exception_function_code = static_cast<uint8_t>(
    kReadHoldingRegistersFunctionCode | 0x80U);
  if (response_bytes.size() == 5U && response_bytes[1] == exception_function_code) {
    std::ostringstream stream;
    stream << "BMS 返回异常码 0x" << std::hex << static_cast<int>(response_bytes[2]);
    error_message = stream.str();
    return false;
  }

  const std::size_t expected_data_bytes = static_cast<std::size_t>(register_count) * 2U;
  const std::size_t expected_frame_size = expected_data_bytes + 5U;
  if (response_bytes.size() != expected_frame_size) {
    std::ostringstream stream;
    stream << "读寄存器响应长度异常，期望 " << expected_frame_size <<
      " 字节，实际 " << response_bytes.size() << " 字节";
    error_message = stream.str();
    return false;
  }

  if (response_bytes[0] != device_address ||
    response_bytes[1] != kReadHoldingRegistersFunctionCode)
  {
    error_message = "读寄存器响应头不匹配";
    return false;
  }

  if (response_bytes[2] != expected_data_bytes) {
    std::ostringstream stream;
    stream << "读寄存器字节数异常，期望 " << expected_data_bytes <<
      "，实际 " << static_cast<int>(response_bytes[2]);
    error_message = stream.str();
    return false;
  }

  register_bytes.assign(response_bytes.begin() + 3,
      response_bytes.begin() + 3 + expected_data_bytes);
  error_message.clear();
  return true;
}

bool SocketCanTransport::exchange_frame(
  const std::vector<uint8_t> & request_bytes,
  std::vector<uint8_t> & response_bytes,
  std::string & error_message)
{
  std::lock_guard<std::mutex> lock(io_mutex_);

  if (!is_connected()) {
    error_message = "CAN 尚未连接";
    return false;
  }

  flush_receive_buffer();

  if (!send_frame(request_bytes, error_message)) {
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms_);
  ModbusResponseAssembler assembler;

  while (true) {
    std::vector<uint8_t> fragment;
    if (!receive_frame(fragment, error_message, deadline)) {
      if (error_message == "等待 BMS 响应超时") {
        error_message = assembler.timeout_error();
      }
      return false;
    }

    const auto status = assembler.append(fragment, error_message);
    if (status == ModbusResponseAssemblyStatus::kError) {
      return false;
    }
    if (status == ModbusResponseAssemblyStatus::kComplete) {
      response_bytes = assembler.bytes();
      return true;
    }
  }
}

bool SocketCanTransport::send_frame(
  const std::vector<uint8_t> & frame_bytes,
  std::string & error_message)
{
  if (frame_bytes.empty() || frame_bytes.size() > 8U) {
    error_message = "CAN 经典帧只支持 1~8 字节 payload";
    return false;
  }

  can_frame frame {};
  frame.can_id = static_cast<canid_t>(can_id_ | (use_extended_frame_ ? CAN_EFF_FLAG : 0U));
  frame.can_dlc = static_cast<__u8>(frame_bytes.size());
  std::memcpy(frame.data, frame_bytes.data(), frame_bytes.size());

  const ssize_t written = write(socket_fd_, &frame, sizeof(frame));
  if (written != static_cast<ssize_t>(sizeof(frame))) {
    error_message = errno_to_string("发送 CAN 帧失败");
    return false;
  }

  return true;
}

bool SocketCanTransport::receive_frame(
  std::vector<uint8_t> & frame_bytes,
  std::string & error_message,
  std::chrono::steady_clock::time_point deadline)
{
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) {
    error_message = "等待 BMS 响应超时";
    return false;
  }

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(socket_fd_, &read_fds);

  const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
  timeval timeout {};
  timeout.tv_sec = static_cast<decltype(timeout.tv_sec)>(remaining.count() / 1000000);
  timeout.tv_usec = static_cast<decltype(timeout.tv_usec)>(remaining.count() % 1000000);

  const int ready = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
  if (ready < 0) {
    error_message = errno_to_string("等待 CAN 响应失败");
    return false;
  }

  if (ready == 0) {
    error_message = "等待 BMS 响应超时";
    return false;
  }

  can_frame frame {};
  const ssize_t received = read(socket_fd_, &frame, sizeof(frame));
  if (received < 0) {
    error_message = errno_to_string("读取 CAN 响应失败");
    return false;
  }

  const bool is_extended_frame = (frame.can_id & CAN_EFF_FLAG) != 0U;
  if (is_extended_frame != use_extended_frame_) {
    error_message = "收到的 CAN 帧类型与配置不一致";
    return false;
  }

  const uint32_t received_can_id = use_extended_frame_ ?
    (frame.can_id & CAN_EFF_MASK) :
    (frame.can_id & CAN_SFF_MASK);

  if (received_can_id != can_id_) {
    error_message = "收到的 CAN ID 与配置不一致";
    return false;
  }

  frame_bytes.assign(frame.data, frame.data + frame.can_dlc);
  return true;
}

void SocketCanTransport::flush_receive_buffer()
{
  if (!is_connected()) {
    return;
  }

  can_frame frame {};
  while (read(socket_fd_, &frame, sizeof(frame)) > 0) {
  }
}

bool SocketCanTransport::validate_crc(const std::vector<uint8_t> & frame_bytes) const
{
  if (frame_bytes.size() < 4U) {
    return false;
  }

  const uint16_t expected_crc = static_cast<uint16_t>(
    frame_bytes[frame_bytes.size() - 2U]) |
    static_cast<uint16_t>(frame_bytes[frame_bytes.size() - 1U] << 8);

  std::vector<uint8_t> payload(frame_bytes.begin(), frame_bytes.end() - 2);
  return crc16(payload) == expected_crc;
}

void SocketCanTransport::append_crc(std::vector<uint8_t> & frame_bytes) const
{
  const uint16_t crc_value = crc16(frame_bytes);
  frame_bytes.push_back(static_cast<uint8_t>(crc_value & 0xFFU));
  frame_bytes.push_back(static_cast<uint8_t>((crc_value >> 8) & 0xFFU));
}

uint16_t SocketCanTransport::crc16(const std::vector<uint8_t> & frame_bytes) const
{
  uint16_t crc = 0xFFFFU;
  for (const uint8_t byte : frame_bytes) {
    crc ^= byte;
    for (int bit_index = 0; bit_index < 8; ++bit_index) {
      if ((crc & 0x0001U) != 0U) {
        crc >>= 1U;
        crc ^= 0xA001U;
      } else {
        crc >>= 1U;
      }
    }
  }
  return crc;
}

}  // namespace tws_battery_driver_ros2
