#ifndef SMIT_ROS_DRIVER__SMIT_LIDAR_DRIVER_HPP_
#define SMIT_ROS_DRIVER__SMIT_LIDAR_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "smit_ros_driver/serial_transport.hpp"
#include "smit_ros_driver/smit_protocol.hpp"

namespace smit_ros_driver
{

struct SmitLidarConfig
{
  std::string port {""};
  int baudrate {921600};
  double angle_min_deg {-180.0};
  double angle_max_deg {180.0};
  std::size_t read_chunk_size {1024};
  int data_timeout_ms {2000};
  int reconnect_backoff_ms {1000};
};

struct SmitLidarSnapshot
{
  bool connected {false};
  bool config_received {false};
  std::string last_error;
  std::uint64_t frame_count {0};
};

/// 一圈（或角度窗口一趟）扫描结果，交给 ROS 层组装 LaserScan
struct SmitScan
{
  std::vector<SmitPoint> points;   ///< angle_deg ∈ [0, 360)
  double angle_min_deg {-180.0};
  double angle_max_deg {180.0};
  int motor_speed {0};             ///< 电机转速 Hz
  int lidar_freq {0};              ///< 点频 points/s
};

/// 点云包 → 整圈扫描装配器（ROS-free）。
/// 角度窗口逻辑移植自旧驱动的 SMiTPub::pub_data：
///   - 满圈窗口（宽度 ≥ 360°）：角度跨过窗口起点边界时完成一圈并上抛；
///   - 部分窗口：角度从窗外进入窗内开始积累，离开窗口时上抛；
///   - 收到 CRC 失败的坏包时丢弃积累的半圈数据；
///   - 未收到配置包（电机转速 / 点频未知）时不产生输出。
class SmitScanAssembler
{
public:
  using ScanCallback = std::function<void(const SmitScan &)>;

  SmitScanAssembler(double angle_min_deg, double angle_max_deg, ScanCallback callback);

  void set_window(double angle_min_deg, double angle_max_deg);
  void set_config(int motor_speed, int lidar_freq);
  void add_packet(const SmitPointPacket & packet);
  void reset();

private:
  void emit_scan();
  void clear_points();

  double angle_min_deg_;
  double angle_max_deg_;
  ScanCallback callback_;

  int motor_speed_ {0};
  int lidar_freq_ {0};
  bool has_config_ {false};

  std::vector<SmitPoint> points_;
  bool collecting_ {false};
  bool have_previous_ {false};
  bool previous_in_window_ {false};
  double previous_raw_angle_deg_ {0.0};
};

class SmitLidarDriver
{
public:
  using TransportFactory =
    std::function<std::unique_ptr<SerialTransport>(const SmitLidarConfig &)>;
  using ScanHandler = std::function<void(const SmitScan &)>;
  using ConfigHandler = std::function<void(const SmitConfigInfo &)>;
  using ConnectionHandler = std::function<void(bool, const std::string &)>;

  explicit SmitLidarDriver(
    SmitLidarConfig config = {},
    TransportFactory transport_factory = {});
  ~SmitLidarDriver();

  bool configure(const SmitLidarConfig & config, std::string & error_message);
  void cleanup();
  bool is_configured() const;

  void start();
  void stop();

  void set_scan_handler(ScanHandler handler);
  void set_config_handler(ConfigHandler handler);
  void set_connection_handler(ConnectionHandler handler);

  SmitLidarSnapshot snapshot() const;
  const SmitLidarConfig & config() const;

private:
  void read_loop();
  void handle_config(const SmitConfigInfo & config);
  void handle_point_packet(const SmitPointPacket & packet);
  void handle_scan(const SmitScan & scan);
  void handle_fault(const std::string & message);
  bool data_stalled() const;
  void interruptible_sleep(int duration_ms);
  void notify_connection(bool connected, const std::string & message);

  SmitLidarConfig config_;
  TransportFactory transport_factory_;
  SmitProtocol protocol_;
  SmitScanAssembler assembler_;

  std::atomic<bool> running_ {false};
  std::thread read_thread_;
  std::unique_ptr<SerialTransport> transport_;

  mutable std::mutex mutex_;
  bool configured_ {false};
  bool connected_ {false};
  bool config_received_ {false};
  std::string last_error_;
  std::uint64_t frame_count_ {0};
  std::chrono::steady_clock::time_point last_frame_time_;
  ScanHandler scan_handler_;
  ConfigHandler config_handler_;
  ConnectionHandler connection_handler_;
};

}  // namespace smit_ros_driver

#endif  // SMIT_ROS_DRIVER__SMIT_LIDAR_DRIVER_HPP_
