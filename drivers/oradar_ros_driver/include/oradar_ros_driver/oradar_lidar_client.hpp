#ifndef ORADAR_ROS_DRIVER__ORADAR_LIDAR_CLIENT_HPP_
#define ORADAR_ROS_DRIVER__ORADAR_LIDAR_CLIENT_HPP_

#include <functional>
#include <memory>
#include <string>

#include "ord/ord_types.h"

namespace oradar_ros_driver
{

struct OradarLidarConfig
{
  std::string ip_address {"192.168.1.100"};
  std::string network_interface;
  int udp_port {2007};
  int motor_speed {15};
  int filter_size {0};
  int motor_dir {0};
  int timeout_ms {3000};
  int reconnect_backoff_ms {1000};
  double angle_min_deg {-135.0};
  double angle_max_deg {135.0};
  double range_min_m {0.05};
  double range_max_m {30.0};
  bool inverted {false};
};

class OradarLidarClient
{
public:
  virtual ~OradarLidarClient() = default;

  virtual ord_sdk::error_t open() = 0;
  virtual bool is_open() const = 0;
  virtual void close() = 0;
  virtual void set_timeout(int timeout_ms) = 0;
  virtual ord_sdk::error_t track_connect() = 0;
  virtual ord_sdk::error_t enable_measure() = 0;
  virtual ord_sdk::error_t disable_measure() = 0;
  virtual ord_sdk::error_t enable_data_stream() = 0;
  virtual ord_sdk::error_t disable_data_stream() = 0;
  virtual ord_sdk::error_t get_scan_speed(std::uint32_t & speed) = 0;
  virtual ord_sdk::error_t set_scan_speed(std::uint32_t speed) = 0;
  virtual ord_sdk::error_t get_tail_filter_level(std::uint32_t & level) = 0;
  virtual ord_sdk::error_t set_tail_filter_level(std::uint32_t level) = 0;
  virtual ord_sdk::error_t get_scan_direction(std::uint32_t & direction) = 0;
  virtual ord_sdk::error_t set_scan_direction(std::uint32_t direction) = 0;
  virtual ord_sdk::error_t apply_configs() = 0;
  virtual ord_sdk::error_t get_scan_frame_data(ord_sdk::ScanFrameData & frame) = 0;
};

using OradarLidarClientFactory =
  std::function<std::unique_ptr<OradarLidarClient>(const OradarLidarConfig &)>;

std::unique_ptr<OradarLidarClient> make_ord_driver_client(const OradarLidarConfig & config);

}  // namespace oradar_ros_driver

#endif  // ORADAR_ROS_DRIVER__ORADAR_LIDAR_CLIENT_HPP_
