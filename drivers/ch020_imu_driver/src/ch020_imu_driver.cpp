#include "ch020_imu_driver/ch020_imu_driver.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ch020_imu_driver/posix_serial_transport.hpp"

namespace ch020_imu_driver
{

Ch020ImuDriver::Ch020ImuDriver(Ch020ImuConfig config, TransportFactory transport_factory)
: config_(std::move(config)),
  transport_factory_(std::move(transport_factory)),
  decoder_([this](const ImuSolPacket & packet) {handle_packet(packet);})
{
}

Ch020ImuDriver::~Ch020ImuDriver()
{
  cleanup();
}

bool Ch020ImuDriver::configure(const Ch020ImuConfig & config, std::string & error_message)
{
  cleanup();
  config_ = config;

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
  last_error_.clear();
  frame_count_ = 0;
  last_frame_time_ = std::chrono::steady_clock::now();
  return true;
}

void Ch020ImuDriver::cleanup()
{
  stop();
  std::lock_guard<std::mutex> lock(mutex_);
  if (transport_) {
    transport_->close();
    transport_.reset();
  }
  configured_ = false;
  connected_ = false;
}

bool Ch020ImuDriver::is_configured() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return configured_;
}

void Ch020ImuDriver::start()
{
  if (running_.exchange(true)) {
    return;
  }
  read_thread_ = std::thread(&Ch020ImuDriver::read_loop, this);
}

void Ch020ImuDriver::stop()
{
  running_.store(false);
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

void Ch020ImuDriver::set_packet_handler(PacketHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  packet_handler_ = std::move(handler);
}

void Ch020ImuDriver::set_connection_handler(ConnectionHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  connection_handler_ = std::move(handler);
}

Ch020ImuSnapshot Ch020ImuDriver::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  Ch020ImuSnapshot snapshot;
  snapshot.connected = connected_;
  snapshot.last_error = last_error_;
  snapshot.frame_count = frame_count_;
  return snapshot;
}

const Ch020ImuConfig & Ch020ImuDriver::config() const
{
  return config_;
}

void Ch020ImuDriver::read_loop()
{
  std::vector<uint8_t> buffer(config_.read_chunk_size);
  while (running_.load()) {
    if (!transport_->is_open()) {
      std::string error_message;
      if (transport_->open(config_.port, config_.baudrate, error_message)) {
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
      decoder_.feed(buffer.data(), static_cast<std::size_t>(received));
    } else if (received < 0) {
      handle_fault("读取串口失败: " + error_message);
      continue;
    }

    if (data_stalled()) {
      handle_fault("串口数据停滞超时");
    }
  }
}

void Ch020ImuDriver::handle_packet(const ImuSolPacket & packet)
{
  PacketHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_frame_time_ = std::chrono::steady_clock::now();
    ++frame_count_;
    handler = packet_handler_;
  }
  if (handler) {
    handler(packet);
  }
}

void Ch020ImuDriver::handle_fault(const std::string & message)
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

bool Ch020ImuDriver::data_stalled() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_) {
    return false;
  }
  const auto elapsed = std::chrono::steady_clock::now() - last_frame_time_;
  return elapsed > std::chrono::milliseconds(config_.data_timeout_ms);
}

void Ch020ImuDriver::interruptible_sleep(int duration_ms)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(duration_ms);
  while (running_.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void Ch020ImuDriver::notify_connection(bool connected, const std::string & message)
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

}  // namespace ch020_imu_driver
