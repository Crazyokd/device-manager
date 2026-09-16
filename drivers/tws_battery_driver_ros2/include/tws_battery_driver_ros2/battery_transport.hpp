#ifndef TWS_BATTERY_DRIVER_ROS2__BATTERY_TRANSPORT_HPP_
#define TWS_BATTERY_DRIVER_ROS2__BATTERY_TRANSPORT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace tws_battery_driver_ros2
{

class BatteryTransport
{
public:
  virtual ~BatteryTransport() = default;

  virtual bool connect(std::string & error_message) = 0;
  virtual void disconnect() = 0;
  virtual bool is_connected() const = 0;

  virtual bool read_holding_registers(
    uint8_t device_address,
    uint16_t register_address,
    uint16_t register_count,
    std::vector<uint8_t> & register_bytes,
    std::string & error_message) = 0;
};

}  // namespace tws_battery_driver_ros2

#endif  // TWS_BATTERY_DRIVER_ROS2__BATTERY_TRANSPORT_HPP_
