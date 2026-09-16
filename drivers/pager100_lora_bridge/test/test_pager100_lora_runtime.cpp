#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "device_manager_core/device_manager.hpp"
#include "pager100_lora_bridge/pager100_lora_runtime.hpp"

namespace pager100_lora_bridge
{
namespace
{

// ---------------------------------------------------------------------------
// 假串口传输：脚本化 open/read/write 行为，供生命周期与断线恢复测试注入
// ---------------------------------------------------------------------------

class FakeSerialTransport final : public SerialTransport
{
public:
  bool open(
    const std::string &,
    int,
    std::string & error_message) override
  {
    ++open_count;
    if (!open_result.load()) {
      error_message = "open failed";
      opened = false;
      return false;
    }
    error_message.clear();
    opened = true;
    return true;
  }

  void close() override
  {
    opened = false;
  }

  bool is_open() const override
  {
    return opened.load();
  }

  int read(
    std::uint8_t * buffer,
    std::size_t max_length,
    std::string & error_message) override
  {
    if (read_error.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      error_message = "read failed";
      return -1;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pending_.empty()) {
        const std::size_t count = std::min(max_length, pending_.size());
        std::memcpy(buffer, pending_.data(), count);
        pending_.erase(pending_.begin(), pending_.begin() + count);
        return static_cast<int>(count);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return 0;
  }

  bool write(
    const std::uint8_t * data,
    std::size_t length,
    std::string & error_message) override
  {
    if (write_error.load()) {
      error_message = "write failed";
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    written_.insert(written_.end(), data, data + length);
    error_message.clear();
    return true;
  }

  void push_bytes(const std::vector<std::uint8_t> & bytes)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.insert(pending_.end(), bytes.begin(), bytes.end());
  }

  std::vector<std::uint8_t> written() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return written_;
  }

  std::atomic<bool> open_result {true};
  std::atomic<bool> read_error {false};
  std::atomic<bool> write_error {false};
  std::atomic<int> open_count {0};

private:
  std::atomic<bool> opened {false};
  mutable std::mutex mutex_;
  std::vector<std::uint8_t> pending_;
  std::vector<std::uint8_t> written_;
};

std::unique_ptr<SerialTransport> make_transport(FakeSerialTransport *& current)
{
  auto transport = std::make_unique<FakeSerialTransport>();
  current = transport.get();
  return transport;
}

device_manager::TransitionRequest transition(
  device_manager::LifecycleState from,
  device_manager::LifecycleState target,
  device_manager::ParameterMap parameters = {})
{
  return device_manager::TransitionRequest{from, target, std::move(parameters)};
}

bool wait_until(const std::function<bool()> & predicate, int timeout_ms = 3000)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

class EventRecorder
{
public:
  device_manager::EventHandler handler()
  {
    return [this](device_manager::DeviceEvent event) {
             std::lock_guard<std::mutex> lock(mutex_);
             events_.push_back(std::move(event));
           };
  }

  bool contains(const std::string & code) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::any_of(
      events_.begin(), events_.end(),
      [&code](const device_manager::DeviceEvent & event) {return event.code == code;});
  }

private:
  mutable std::mutex mutex_;
  std::vector<device_manager::DeviceEvent> events_;
};

TEST(Pager100LoraRuntime, UsesAuthoritativeDeviceParameters)
{
  Pager100LoraConfig applied;
  Pager100LoraRuntime runtime(
    Pager100LoraConfig{},
    [&applied](const Pager100LoraConfig & config) {
      applied = config;
      return std::make_unique<FakeSerialTransport>();
    });

  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive,
        {{"device.interface.serial_port", std::string{"/dev/ttyUSB9"}},
          {"device.interface.serial_baudrate", std::int64_t{9600}}})).result,
    device_manager::TransitionResult::kSuccess);

  EXPECT_EQ(applied.port, "/dev/ttyUSB9");
  EXPECT_EQ(applied.baudrate, 9600);
}

TEST(Pager100LoraRuntime, MaterializesDriverOnDemand)
{
  FakeSerialTransport * transport = nullptr;
  int transport_count = 0;
  Pager100LoraRuntime runtime(
    Pager100LoraConfig{},
    [&transport, &transport_count](const Pager100LoraConfig &) {
      ++transport_count;
      return make_transport(transport);
    });

  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kFinalized);
  EXPECT_FALSE(runtime.materialized());
  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_TRUE(runtime.materialized());
  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(transport_count, 1);

  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kFinalized)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_FALSE(runtime.materialized());
}

TEST(Pager100LoraRuntime, EmitsConnectFailedAndRetriesViaHook)
{
  // 工厂在 configure 时才运行；用捕获的 open_result 控制每次新开串口的成败。
  std::atomic<bool> open_result {false};
  FakeSerialTransport * transport = nullptr;
  Pager100LoraRuntime runtime(
    Pager100LoraConfig{},
    [&transport, &open_result](const Pager100LoraConfig &) {
      auto fake = std::make_unique<FakeSerialTransport>();
      transport = fake.get();
      fake->open_result.store(open_result.load());
      return fake;
    });
  EventRecorder recorder;
  runtime.set_event_handler(recorder.handler());

  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kError);
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kUnconfigured);
  EXPECT_TRUE(recorder.contains("pager100.connect_failed"));

  open_result.store(true);
  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kInactive);
}

TEST(Pager100LoraRuntime, ForwardsRxFramesAndReconnectsAfterReadError)
{
  FakeSerialTransport * transport = nullptr;
  Pager100LoraRuntime runtime(
    Pager100LoraConfig{},
    [&transport](const Pager100LoraConfig &) {
      return make_transport(transport);
    });
  EventRecorder recorder;
  runtime.set_event_handler(recorder.handler());

  std::mutex frames_mutex;
  std::vector<std::vector<std::uint8_t>> frames;
  runtime.set_frame_handler(
    [&frames_mutex, &frames](const std::vector<std::uint8_t> & payload) {
      std::lock_guard<std::mutex> lock(frames_mutex);
      frames.push_back(payload);
    });

  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);

  const std::vector<std::uint8_t> payload {0x01, 0x02};
  transport->push_bytes(encode_frame(payload, 0));
  ASSERT_TRUE(
    wait_until([&frames_mutex, &frames]() {
      std::lock_guard<std::mutex> lock(frames_mutex);
      return !frames.empty();
    }));
  {
    std::lock_guard<std::mutex> lock(frames_mutex);
    EXPECT_EQ(frames.front(), payload);
  }

  // 读错误 → disconnected 事件；串口按退避重开 → online 事件。
  // （假串口重开即时成功，connected 快照存在竞争，只断言事件契约。）
  transport->read_error.store(true);
  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("pager100.disconnected");}));

  transport->read_error.store(false);
  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("pager100.online");}));
  ASSERT_TRUE(wait_until([&runtime]() {return runtime.snapshot().connected;}));

  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kActive,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kFinalized)).result,
    device_manager::TransitionResult::kSuccess);
}

TEST(Pager100LoraDriver, SendEncodesFrameAndTracksSequence)
{
  FakeSerialTransport * transport = nullptr;
  Pager100LoraDriver driver(
    Pager100LoraConfig{},
    [&transport](const Pager100LoraConfig &) {
      return make_transport(transport);
    });

  std::string error_message;
  ASSERT_TRUE(driver.configure(Pager100LoraConfig{}, error_message)) << error_message;

  ASSERT_TRUE(driver.send({0xAA}, error_message)) << error_message;
  ASSERT_TRUE(driver.send({0xBB}, error_message)) << error_message;

  const auto written = transport->written();
  const auto first = encode_frame({0xAA}, 0);
  const auto second = encode_frame({0xBB}, 1);
  std::vector<std::uint8_t> expected(first.begin(), first.end());
  expected.insert(expected.end(), second.begin(), second.end());
  EXPECT_EQ(written, expected);

  driver.cleanup();
}

TEST(Pager100LoraDriver, SendFailsOnClosedPortAndOversizePayload)
{
  FakeSerialTransport * transport = nullptr;
  Pager100LoraDriver driver(
    Pager100LoraConfig{},
    [&transport](const Pager100LoraConfig &) {
      return make_transport(transport);
    });

  std::string error_message;
  EXPECT_FALSE(driver.send({0x01}, error_message));

  ASSERT_TRUE(driver.configure(Pager100LoraConfig{}, error_message)) << error_message;
  EXPECT_FALSE(driver.send(std::vector<std::uint8_t>(kMaxPayloadSize + 1), error_message));

  transport->write_error.store(true);
  EXPECT_FALSE(driver.send({0x01}, error_message));
  EXPECT_FALSE(driver.snapshot().connected);

  driver.cleanup();
}

}  // namespace
}  // namespace pager100_lora_bridge
