#include "pager100_lora_bridge/pager100_lora_driver.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "pager100_lora_bridge/posix_serial_transport.hpp"

namespace pager100_lora_bridge
{

Pager100LoraDriver::Pager100LoraDriver(
  Pager100LoraConfig config,
  TransportFactory transport_factory)
: config_(std::move(config)),
  transport_factory_(std::move(transport_factory))
{
}

Pager100LoraDriver::~Pager100LoraDriver()
{
  cleanup();
}

bool Pager100LoraDriver::configure(
  const Pager100LoraConfig & config,
  std::string & error_message)
{
  cleanup();
  config_ = config;

  auto transport = transport_factory_ ?
    transport_factory_(config_) :
    std::unique_ptr<SerialTransport>(std::make_unique<PosixSerialTransport>());

  if (!transport->open(config_.port, config_.baudrate, error_message)) {
    std::lock_guard<std::mutex> lock(mutex_);
    configured_ = false;
    connected_ = false;
    last_error_ = error_message;
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  transport_ = std::move(transport);
  configured_ = true;
  connected_ = true;
  last_error_.clear();
  frame_count_ = 0;
  tx_sequence_ = 0;
  return true;
}

void Pager100LoraDriver::cleanup()
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

bool Pager100LoraDriver::is_configured() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return configured_;
}

void Pager100LoraDriver::start()
{
  if (running_.exchange(true)) {
    return;
  }
  read_thread_ = std::thread(&Pager100LoraDriver::read_loop, this);
}

void Pager100LoraDriver::stop()
{
  running_.store(false);
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

bool Pager100LoraDriver::send(
  const std::vector<std::uint8_t> & payload,
  std::string & error_message)
{
  std::vector<std::uint8_t> frame;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!transport_ || !transport_->is_open()) {
      error_message = "串口尚未打开";
      return false;
    }

    try {
      frame = encode_frame(payload, tx_sequence_, kFrameTypeData);
    } catch (const std::invalid_argument & error) {
      error_message = error.what();
      return false;
    }

    if (!transport_->write(frame.data(), frame.size(), error_message)) {
      // 写失败按断线处理，读线程侧按退避间隔重开串口。
      transport_->close();
      connected_ = false;
      last_error_ = error_message;
      return false;
    }

    tx_sequence_ = static_cast<std::uint8_t>((tx_sequence_ + 1) & 0xFF);
  }

  error_message.clear();
  return true;
}

void Pager100LoraDriver::set_frame_handler(FrameHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  frame_handler_ = std::move(handler);
}

void Pager100LoraDriver::set_connection_handler(ConnectionHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  connection_handler_ = std::move(handler);
}

Pager100LoraSnapshot Pager100LoraDriver::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  Pager100LoraSnapshot snapshot;
  snapshot.connected = connected_;
  snapshot.last_error = last_error_;
  snapshot.frame_count = frame_count_;
  return snapshot;
}

const Pager100LoraConfig & Pager100LoraDriver::config() const
{
  return config_;
}

void Pager100LoraDriver::read_loop()
{
  // transport_ 的读路径由本线程独占；cleanup() 先 stop() join 本线程再释放
  // transport_，因此这里无需持锁访问。send() 只做 write，POSIX 全双工安全。
  std::vector<std::uint8_t> buffer(config_.read_chunk_size);
  while (running_.load()) {
    if (!transport_->is_open()) {
      std::string error_message;
      if (transport_->open(config_.port, config_.baudrate, error_message)) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          connected_ = true;
          last_error_.clear();
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
      for (const auto & frame : decoder_.feed(buffer.data(), static_cast<std::size_t>(received))) {
        if (frame.frame_type == kFrameTypeData) {
          handle_frame(frame);
        }
      }
    } else if (received < 0) {
      handle_fault("读取串口失败: " + error_message);
    }
  }
}

void Pager100LoraDriver::handle_frame(const SerialFrame & frame)
{
  FrameHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++frame_count_;
    handler = frame_handler_;
  }
  if (handler) {
    handler(frame.payload);
  }
}

void Pager100LoraDriver::handle_fault(const std::string & message)
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

void Pager100LoraDriver::interruptible_sleep(int duration_ms)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(duration_ms);
  while (running_.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void Pager100LoraDriver::notify_connection(bool connected, const std::string & message)
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

}  // namespace pager100_lora_bridge
