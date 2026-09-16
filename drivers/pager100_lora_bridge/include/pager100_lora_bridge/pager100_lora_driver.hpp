#ifndef PAGER100_LORA_BRIDGE__PAGER100_LORA_DRIVER_HPP_
#define PAGER100_LORA_BRIDGE__PAGER100_LORA_DRIVER_HPP_

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

#include "pager100_lora_bridge/serial_frame.hpp"
#include "pager100_lora_bridge/serial_transport.hpp"

namespace pager100_lora_bridge
{

struct Pager100LoraConfig
{
  std::string port {"/dev/ttyS6"};
  int baudrate {115200};
  std::size_t read_chunk_size {128};
  int reconnect_backoff_ms {1000};
};

struct Pager100LoraSnapshot
{
  bool connected {false};
  std::string last_error;
  std::uint64_t frame_count {0};
};

/// PG100 呼叫器车端 LoRa 串口桥：读线程 + 断线自愈 + 数据帧透传。
///
/// 与 IMU/雷达不同，LoRa 链路空闲是常态（没人呼叫时没有任何流量），
/// 因此连接状态只依据串口读写健康判断，不做数据停滞超时判定——
/// 无法区分“模块故障”和“无人呼叫”。
class Pager100LoraDriver
{
public:
  using TransportFactory =
    std::function<std::unique_ptr<SerialTransport>(const Pager100LoraConfig &)>;
  using FrameHandler = std::function<void(const std::vector<std::uint8_t> &)>;
  using ConnectionHandler = std::function<void(bool, const std::string &)>;

  explicit Pager100LoraDriver(
    Pager100LoraConfig config = {},
    TransportFactory transport_factory = {});
  ~Pager100LoraDriver();

  bool configure(const Pager100LoraConfig & config, std::string & error_message);
  void cleanup();
  bool is_configured() const;

  void start();
  void stop();

  /// 编码并下发一帧数据；串口不可用或写入失败时返回 false（error_message 已填写）。
  bool send(const std::vector<std::uint8_t> & payload, std::string & error_message);

  void set_frame_handler(FrameHandler handler);
  void set_connection_handler(ConnectionHandler handler);

  Pager100LoraSnapshot snapshot() const;
  const Pager100LoraConfig & config() const;

private:
  void read_loop();
  void handle_frame(const SerialFrame & frame);
  void handle_fault(const std::string & message);
  void interruptible_sleep(int duration_ms);
  void notify_connection(bool connected, const std::string & message);

  Pager100LoraConfig config_;
  TransportFactory transport_factory_;
  SerialFrameDecoder decoder_;

  std::atomic<bool> running_ {false};
  std::thread read_thread_;

  mutable std::mutex mutex_;
  std::unique_ptr<SerialTransport> transport_;
  bool configured_ {false};
  bool connected_ {false};
  std::string last_error_;
  std::uint64_t frame_count_ {0};
  std::uint8_t tx_sequence_ {0};
  FrameHandler frame_handler_;
  ConnectionHandler connection_handler_;
};

}  // namespace pager100_lora_bridge

#endif  // PAGER100_LORA_BRIDGE__PAGER100_LORA_DRIVER_HPP_
