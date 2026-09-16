#ifndef TWS_BATTERY_DRIVER_ROS2__SOCKETCAN_TRANSPORT_HPP_
#define TWS_BATTERY_DRIVER_ROS2__SOCKETCAN_TRANSPORT_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "tws_battery_driver_ros2/battery_transport.hpp"

namespace tws_battery_driver_ros2
{

class SocketCanTransport final : public BatteryTransport
{
public:
  SocketCanTransport(
    const std::string & interface_name,
    uint32_t can_id,
    bool use_extended_frame,
    int timeout_ms);
  ~SocketCanTransport() override;

  bool connect(std::string & error_message) override;
  void disconnect() override;
  bool is_connected() const override;

  bool read_holding_registers(
    uint8_t device_address,
    uint16_t register_address,
    uint16_t register_count,
    std::vector<uint8_t> & register_bytes,
    std::string & error_message) override;

private:
  bool exchange_frame(
    const std::vector<uint8_t> & request_bytes,
    std::vector<uint8_t> & response_bytes,
    std::string & error_message);

  bool send_frame(const std::vector<uint8_t> & frame_bytes, std::string & error_message);
  bool receive_frame(
    std::vector<uint8_t> & frame_bytes,
    std::string & error_message,
    std::chrono::steady_clock::time_point deadline);
  void flush_receive_buffer();
  bool validate_crc(const std::vector<uint8_t> & frame_bytes) const;
  void append_crc(std::vector<uint8_t> & frame_bytes) const;
  uint16_t crc16(const std::vector<uint8_t> & frame_bytes) const;

  std::string interface_name_;
  uint32_t can_id_;
  bool use_extended_frame_;
  int timeout_ms_;
  int socket_fd_;
  mutable std::mutex io_mutex_;
};

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__SOCKETCAN_TRANSPORT_HPP_
