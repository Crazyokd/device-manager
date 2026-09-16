#include "smit_ros_driver/smit_lidar_driver.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "smit_ros_driver/posix_serial_transport.hpp"

namespace smit_ros_driver
{

namespace
{

constexpr double kFullCircleDegrees = 360.0;
constexpr double kFullCircleToleranceDegrees = 1e-3;

double normalize_raw_angle(double angle_deg)
{
  double normalized = std::fmod(angle_deg, kFullCircleDegrees);
  if (normalized < 0.0) {
    normalized += kFullCircleDegrees;
  }
  return normalized;
}

bool is_full_circle_window(double angle_min_deg, double angle_max_deg)
{
  return angle_max_deg - angle_min_deg >=
         kFullCircleDegrees - kFullCircleToleranceDegrees;
}

bool is_raw_angle_in_window(
  double raw_angle_deg, double start_raw_deg,
  double end_raw_deg, bool full_circle)
{
  if (full_circle) {
    return true;
  }
  const double normalized = normalize_raw_angle(raw_angle_deg);
  if (start_raw_deg <= end_raw_deg) {
    return normalized >= start_raw_deg && normalized <= end_raw_deg;
  }
  return normalized >= start_raw_deg || normalized <= end_raw_deg;
}

bool did_raw_angle_cross_boundary(
  double previous_raw_deg, double current_raw_deg,
  double boundary_raw_deg)
{
  if (previous_raw_deg <= current_raw_deg) {
    return previous_raw_deg < boundary_raw_deg &&
           current_raw_deg >= boundary_raw_deg;
  }
  return previous_raw_deg < boundary_raw_deg ||
         current_raw_deg >= boundary_raw_deg;
}

}  // namespace

// ---------------------------------------------------------------------------
// SmitScanAssembler
// ---------------------------------------------------------------------------

SmitScanAssembler::SmitScanAssembler(
  double angle_min_deg, double angle_max_deg, ScanCallback callback)
: angle_min_deg_(angle_min_deg),
  angle_max_deg_(angle_max_deg),
  callback_(std::move(callback))
{
}

void SmitScanAssembler::set_window(double angle_min_deg, double angle_max_deg)
{
  angle_min_deg_ = angle_min_deg;
  angle_max_deg_ = angle_max_deg;
  reset();
}

void SmitScanAssembler::set_config(int motor_speed, int lidar_freq)
{
  motor_speed_ = motor_speed;
  lidar_freq_ = lidar_freq;
  has_config_ = true;
}

void SmitScanAssembler::reset()
{
  clear_points();
  collecting_ = false;
  have_previous_ = false;
  previous_in_window_ = false;
}

void SmitScanAssembler::clear_points()
{
  points_.clear();
}

void SmitScanAssembler::emit_scan()
{
  SmitScan scan;
  scan.points = points_;
  scan.angle_min_deg = angle_min_deg_;
  scan.angle_max_deg = angle_max_deg_;
  scan.motor_speed = motor_speed_;
  scan.lidar_freq = lidar_freq_;
  callback_(scan);
}

void SmitScanAssembler::add_packet(const SmitPointPacket & packet)
{
  if (!has_config_) {
    // 旧驱动在拿到配置包前 pub 线程整体阻塞；这里等价地丢弃点云直到配置就绪
    return;
  }
  if (!packet.crc_ok || packet.point_count > SmitPointPacket::kMaxPoints) {
    reset();
    return;
  }

  const bool full_circle = is_full_circle_window(angle_min_deg_, angle_max_deg_);
  const double start_raw_deg = normalize_raw_angle(angle_min_deg_);
  const double end_raw_deg = normalize_raw_angle(angle_max_deg_);

  for (std::size_t i = 0; i < packet.point_count; ++i) {
    const SmitPoint & point = packet.points[i];
    const double raw_angle_deg = normalize_raw_angle(point.angle_deg);

    if (full_circle) {
      if (have_previous_ &&
        did_raw_angle_cross_boundary(previous_raw_angle_deg_, raw_angle_deg, start_raw_deg))
      {
        if (collecting_ && !points_.empty()) {
          emit_scan();
        }
        clear_points();
        collecting_ = true;
      }
      if (collecting_) {
        points_.push_back(
          SmitPoint{static_cast<float>(raw_angle_deg), point.distance_m, point.intensity});
      }
      previous_raw_angle_deg_ = raw_angle_deg;
      have_previous_ = true;
      continue;
    }

    const bool in_window =
      is_raw_angle_in_window(raw_angle_deg, start_raw_deg, end_raw_deg, false);
    if (!collecting_) {
      if (have_previous_ && !previous_in_window_ && in_window) {
        clear_points();
        collecting_ = true;
        points_.push_back(
          SmitPoint{static_cast<float>(raw_angle_deg), point.distance_m, point.intensity});
      }
    } else if (!in_window) {
      if (!points_.empty()) {
        emit_scan();
      }
      clear_points();
      collecting_ = false;
    } else {
      points_.push_back(
        SmitPoint{static_cast<float>(raw_angle_deg), point.distance_m, point.intensity});
    }

    previous_raw_angle_deg_ = raw_angle_deg;
    previous_in_window_ = in_window;
    have_previous_ = true;
  }
}

// ---------------------------------------------------------------------------
// SmitLidarDriver
// ---------------------------------------------------------------------------

SmitLidarDriver::SmitLidarDriver(SmitLidarConfig config, TransportFactory transport_factory)
: config_(std::move(config)),
  transport_factory_(std::move(transport_factory)),
  protocol_(
    [this](const SmitConfigInfo & config) {handle_config(config);},
    [this](const SmitPointPacket & packet) {handle_point_packet(packet);}),
  assembler_(
    config_.angle_min_deg, config_.angle_max_deg,
    [this](const SmitScan & scan) {handle_scan(scan);})
{
}

SmitLidarDriver::~SmitLidarDriver()
{
  cleanup();
}

bool SmitLidarDriver::configure(const SmitLidarConfig & config, std::string & error_message)
{
  cleanup();
  config_ = config;
  assembler_.set_window(config_.angle_min_deg, config_.angle_max_deg);

  transport_ = transport_factory_ ?
    transport_factory_(config_) :
    std::make_unique<PosixSerialTransport>();

  if (!transport_->open(config_.port, config_.baudrate, error_message)) {
    transport_.reset();
    std::lock_guard<std::mutex> lock(mutex_);
    configured_ = false;
    connected_ = false;
    last_error_ = error_message;
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  configured_ = true;
  connected_ = true;
  config_received_ = false;
  last_error_.clear();
  frame_count_ = 0;
  last_frame_time_ = std::chrono::steady_clock::now();
  return true;
}

void SmitLidarDriver::cleanup()
{
  stop();
  std::lock_guard<std::mutex> lock(mutex_);
  if (transport_) {
    transport_->close();
    transport_.reset();
  }
  configured_ = false;
  connected_ = false;
  config_received_ = false;
}

bool SmitLidarDriver::is_configured() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return configured_;
}

void SmitLidarDriver::start()
{
  if (running_.exchange(true)) {
    return;
  }
  read_thread_ = std::thread(&SmitLidarDriver::read_loop, this);
}

void SmitLidarDriver::stop()
{
  running_.store(false);
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

void SmitLidarDriver::set_scan_handler(ScanHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  scan_handler_ = std::move(handler);
}

void SmitLidarDriver::set_config_handler(ConfigHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_handler_ = std::move(handler);
}

void SmitLidarDriver::set_connection_handler(ConnectionHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  connection_handler_ = std::move(handler);
}

SmitLidarSnapshot SmitLidarDriver::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  SmitLidarSnapshot snapshot;
  snapshot.connected = connected_;
  snapshot.config_received = config_received_;
  snapshot.last_error = last_error_;
  snapshot.frame_count = frame_count_;
  return snapshot;
}

const SmitLidarConfig & SmitLidarDriver::config() const
{
  return config_;
}

void SmitLidarDriver::read_loop()
{
  std::vector<uint8_t> buffer(config_.read_chunk_size);
  while (running_.load()) {
    if (!transport_->is_open()) {
      std::string error_message;
      if (transport_->open(config_.port, config_.baudrate, error_message)) {
        assembler_.reset();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          connected_ = true;
          last_error_.clear();
          last_frame_time_ = std::chrono::steady_clock::now();
        }
        notify_connection(true, "串口已恢复连接");
      } else {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          last_error_ = error_message;
        }
        interruptible_sleep(config_.reconnect_backoff_ms);
        continue;
      }
    }

    std::string error_message;
    const int received = transport_->read(buffer.data(), buffer.size(), error_message);
    if (!running_.load()) {
      break;
    }
    if (received > 0) {
      protocol_.feed(buffer.data(), static_cast<std::size_t>(received));
    } else if (received < 0) {
      handle_fault("读取串口失败: " + error_message);
      continue;
    }

    if (data_stalled()) {
      handle_fault("串口数据停滞超时");
    }
  }
}

void SmitLidarDriver::handle_config(const SmitConfigInfo & config)
{
  ConfigHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_frame_time_ = std::chrono::steady_clock::now();
    ++frame_count_;
    config_received_ = true;
    handler = config_handler_;
  }
  // assembler_ 只在读线程上使用，无需持锁
  assembler_.set_config(config.motor_speed, config.lidar_freq);
  if (handler) {
    handler(config);
  }
}

void SmitLidarDriver::handle_point_packet(const SmitPointPacket & packet)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_frame_time_ = std::chrono::steady_clock::now();
    ++frame_count_;
  }
  assembler_.add_packet(packet);
}

void SmitLidarDriver::handle_scan(const SmitScan & scan)
{
  ScanHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handler = scan_handler_;
  }
  if (handler) {
    handler(scan);
  }
}

void SmitLidarDriver::handle_fault(const std::string & message)
{
  transport_->close();
  bool notify = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    notify = connected_;
    connected_ = false;
    last_error_ = message;
  }
  if (notify) {
    notify_connection(false, message);
  }
}

bool SmitLidarDriver::data_stalled() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_) {
    return false;
  }
  const auto elapsed = std::chrono::steady_clock::now() - last_frame_time_;
  return elapsed > std::chrono::milliseconds(config_.data_timeout_ms);
}

void SmitLidarDriver::interruptible_sleep(int duration_ms)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(duration_ms);
  while (running_.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void SmitLidarDriver::notify_connection(bool connected, const std::string & message)
{
  ConnectionHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handler = connection_handler_;
  }
  if (handler) {
    handler(connected, message);
  }
}

}  // namespace smit_ros_driver
