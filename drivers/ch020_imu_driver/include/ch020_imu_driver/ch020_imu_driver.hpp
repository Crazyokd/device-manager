#ifndef CH020_IMU_DRIVER__CH020_IMU_DRIVER_HPP_
#define CH020_IMU_DRIVER__CH020_IMU_DRIVER_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "ch020_imu_driver/hipnuc_decoder.hpp"
#include "ch020_imu_driver/serial_transport.hpp"

namespace ch020_imu_driver
{

struct Ch020ImuConfig
{
  std::string port {"/dev/ttyS7"};
  int baudrate {115200};
  std::size_t read_chunk_size {256};
  int data_timeout_ms {2000};
  int reconnect_backoff_ms {1000};
};

struct Ch020ImuSnapshot
{
  bool connected {false};
  std::string last_error;
  std::uint64_t frame_count {0};
};

class Ch020ImuDriver
{
public:
  using TransportFactory =
    std::function<std::unique_ptr<SerialTransport>(const Ch020ImuConfig &)>;
  using PacketHandler = std::function<void(const ImuSolPacket &)>;
  using ConnectionHandler = std::function<void(bool, const std::string &)>;

  explicit Ch020ImuDriver(
    Ch020ImuConfig config = {},
    TransportFactory transport_factory = {});
  ~Ch020ImuDriver();

  bool configure(const Ch020ImuConfig & config, std::string & error_message);
  void cleanup();
  bool is_configured() const;

  void start();
  void stop();

  void set_packet_handler(PacketHandler handler);
  void set_connection_handler(ConnectionHandler handler);

  Ch020ImuSnapshot snapshot() const;
  const Ch020ImuConfig & config() const;

private:
  void read_loop();
  void handle_packet(const ImuSolPacket & packet);
  void handle_fault(const std::string & message);
  bool data_stalled() const;
  void interruptible_sleep(int duration_ms);
  void notify_connection(bool connected, const std::string & message);

  Ch020ImuConfig config_;
  TransportFactory transport_factory_;
  HipnucDecoder decoder_;

  std::atomic<bool> running_ {false};
  std::thread read_thread_;
  std::unique_ptr<SerialTransport> transport_;

  mutable std::mutex mutex_;
  bool configured_ {false};
  bool connected_ {false};
  std::string last_error_;
  std::uint64_t frame_count_ {0};
  std::chrono::steady_clock::time_point last_frame_time_;
  PacketHandler packet_handler_;
  ConnectionHandler connection_handler_;
};

}  // namespace ch020_imu_driver

#endif  // CH020_IMU_DRIVER__CH020_IMU_DRIVER_HPP_
