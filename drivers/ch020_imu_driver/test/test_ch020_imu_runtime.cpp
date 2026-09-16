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
#include "ch020_imu_driver/ch020_imu_runtime.hpp"

namespace ch020_imu_driver
{
namespace
{

// ---------------------------------------------------------------------------
// 假串口传输：脚本化 open/read 行为，供生命周期与断线恢复测试注入
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
    uint8_t * buffer,
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

  void push_bytes(const std::vector<uint8_t> & bytes)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.insert(pending_.end(), bytes.begin(), bytes.end());
  }

  std::atomic<bool> open_result {true};
  std::atomic<bool> read_error {false};
  std::atomic<int> open_count {0};

private:
  std::atomic<bool> opened {false};
  std::mutex mutex_;
  std::vector<uint8_t> pending_;
};

// ---------------------------------------------------------------------------
// 构造一帧合法 0x91 IMUSOL 数据（与解码器测试同一套格式）
// ---------------------------------------------------------------------------

uint16_t crc16(const uint8_t * data, std::size_t len)
{
  uint16_t crc = 0;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int j = 0; j < 8; ++j) {
      uint16_t t = static_cast<uint16_t>(crc << 1);
      if ((crc & 0x8000U) != 0U) {
        t ^= 0x1021U;
      }
      crc = t;
    }
  }
  return crc;
}

std::vector<uint8_t> build_imusol_frame()
{
  std::vector<uint8_t> payload(76, 0);
  payload[0] = 0x91;
  const float acc_z = 1.0F;
  std::memcpy(payload.data() + 20, &acc_z, 4);
  const float quat_w = 1.0F;
  std::memcpy(payload.data() + 60, &quat_w, 4);

  const uint8_t ll = static_cast<uint8_t>(payload.size() & 0xFFU);
  const uint8_t lh = static_cast<uint8_t>((payload.size() >> 8) & 0xFFU);
  std::vector<uint8_t> crc_in = {0x5A, 0xA5, ll, lh};
  crc_in.insert(crc_in.end(), payload.begin(), payload.end());
  const uint16_t crc = crc16(crc_in.data(), crc_in.size());

  std::vector<uint8_t> frame = {
    0x5A, 0xA5, ll, lh,
    static_cast<uint8_t>(crc & 0xFF),
    static_cast<uint8_t>(crc >> 8)};
  frame.insert(frame.end(), payload.begin(), payload.end());
  return frame;
}

// ---------------------------------------------------------------------------

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

device_manager::HookContext hook_context(
  device_manager::LifecycleState state,
  std::optional<device_manager::DeviceEvent> event = std::nullopt,
  std::uint64_t revision = 0U,
  bool enabled = true)
{
  device_manager::HookContext context;
  context.now = device_manager::Clock::now();
  context.state = state;
  context.enabled = enabled;
  context.latest_event = std::move(event);
  context.latest_event_revision = revision;
  return context;
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

  std::vector<device_manager::DeviceEvent> events() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }

private:
  mutable std::mutex mutex_;
  std::vector<device_manager::DeviceEvent> events_;
};

TEST(Ch020ImuRuntime, UsesAuthoritativeDeviceParameters)
{
  Ch020ImuConfig applied;
  Ch020ImuRuntime runtime(
    Ch020ImuConfig{},
    [&applied](const Ch020ImuConfig & config) {
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
          {"device.interface.serial_baudrate", std::int64_t{460800}}})).result,
    device_manager::TransitionResult::kSuccess);

  EXPECT_EQ(applied.port, "/dev/ttyUSB9");
  EXPECT_EQ(applied.baudrate, 460800);
}

TEST(Ch020ImuRuntime, MaterializesDisabledDriverOnDemand)
{
  FakeSerialTransport * transport = nullptr;
  int transport_count = 0;
  Ch020ImuRuntime runtime(
    Ch020ImuConfig{},
    [&transport, &transport_count](const Ch020ImuConfig &) {
      ++transport_count;
      return make_transport(transport);
    });

  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kFinalized);
  EXPECT_FALSE(runtime.materialized());
  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_TRUE(runtime.materialized());
  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(transport_count, 1);

  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kFinalized)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_FALSE(runtime.materialized());
  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_TRUE(runtime.materialized());
}

TEST(Ch020ImuRuntime, ImplementsBoundedLifecycleTransitions)
{
  FakeSerialTransport * transport = nullptr;
  Ch020ImuRuntime runtime(
    Ch020ImuConfig{},
    [&transport](const Ch020ImuConfig &) {
      return make_transport(transport);
    });

  std::atomic<int> packet_count {0};
  runtime.set_packet_handler(
    [&packet_count](const ImuSolPacket &) {
      ++packet_count;
    });

  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kFinalized);
  EXPECT_FALSE(runtime.materialized());
  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kUnconfigured);
  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_NE(transport, nullptr);
  EXPECT_TRUE(transport->is_open());
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kInactive);

  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kActive);

  transport->push_bytes(build_imusol_frame());
  ASSERT_TRUE(wait_until([&packet_count]() {return packet_count.load() == 1;}));
  EXPECT_TRUE(runtime.snapshot().connected);
  EXPECT_EQ(runtime.snapshot().frame_count, 1U);

  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kActive,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kInactive);

  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_FALSE(transport->is_open());
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kUnconfigured);
}

TEST(Ch020ImuRuntime, DeviceManagerAutoStartsEnabledRuntime)
{
  FakeSerialTransport * transport = nullptr;
  auto runtime = std::make_unique<Ch020ImuRuntime>(
    Ch020ImuConfig{},
    [&transport](const Ch020ImuConfig &) {return make_transport(transport);});
  runtime->set_packet_handler([](const ImuSolPacket &) {});
  auto * runtime_ptr = runtime.get();
  EXPECT_EQ(runtime_ptr->state(), device_manager::LifecycleState::kFinalized);
  EXPECT_FALSE(runtime_ptr->materialized());
  std::vector<device_manager::DeviceRegistration> registrations;
  registrations.push_back(
    {device_manager::DeviceDefinition{"imu", "imu", true, {}}, std::move(runtime)});
  device_manager::DeviceManager manager(std::move(registrations));
  manager.start();

  for (const auto state : {
        device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive})
  {
    manager.tick();
    ASSERT_TRUE(wait_until([runtime_ptr, state]() {return runtime_ptr->state() == state;}));
  }
  EXPECT_NE(transport, nullptr);
  manager.stop();
}

TEST(Ch020ImuRuntime, EmitsConnectFailedEventWhenOpenFails)
{
  FakeSerialTransport * transport = nullptr;
  Ch020ImuRuntime runtime(
    Ch020ImuConfig{},
    [&transport](const Ch020ImuConfig &) {
      auto fake = make_transport(transport);
      static_cast<FakeSerialTransport &>(*fake).open_result = false;
      return fake;
    });
  EventRecorder recorder;
  runtime.set_event_handler(recorder.handler());

  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);
  const auto outcome = runtime.request_transition(
    transition(device_manager::LifecycleState::kUnconfigured,
    device_manager::LifecycleState::kInactive));
  EXPECT_EQ(outcome.result, device_manager::TransitionResult::kError);
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kUnconfigured);

  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("ch020_imu.connect_failed");}));
  const auto events = recorder.events();
  EXPECT_EQ(events.back().level, device_manager::EventLevel::kError);
  EXPECT_EQ(events.back().code, "ch020_imu.connect_failed");
  EXPECT_EQ(events.back().message, "open failed");
  EXPECT_EQ(events.back().source, "ch020_imu_driver");
  EXPECT_EQ(events.back().target_state, device_manager::LifecycleState::kUnconfigured);
}

TEST(Ch020ImuRuntime, RegistersOneHookForAutoStartAndRecovery)
{
  FakeSerialTransport * transport = nullptr;
  Ch020ImuRuntime runtime(
    Ch020ImuConfig{},
    [&transport](const Ch020ImuConfig &) {
      return make_transport(transport);
    });
  device_manager::LifecycleHook hook;
  runtime.register_hook(
    [&hook](device_manager::LifecycleHook registered_hook) {
      hook = std::move(registered_hook);
    });
  ASSERT_TRUE(static_cast<bool>(hook));

  auto requests = hook(hook_context(device_manager::LifecycleState::kFinalized));
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().from_state, device_manager::LifecycleState::kFinalized);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kUnconfigured);

  requests = hook(hook_context(device_manager::LifecycleState::kUnconfigured));
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().from_state, device_manager::LifecycleState::kUnconfigured);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kInactive);

  requests = hook(hook_context(device_manager::LifecycleState::kInactive));
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().from_state, device_manager::LifecycleState::kInactive);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kActive);

  requests = hook(hook_context(device_manager::LifecycleState::kActive, std::nullopt, 0U, false));
  ASSERT_EQ(requests.size(), 3U);
  EXPECT_EQ(requests[0].target_state, device_manager::LifecycleState::kInactive);
  EXPECT_EQ(requests[1].target_state, device_manager::LifecycleState::kUnconfigured);
  EXPECT_EQ(requests[2].target_state, device_manager::LifecycleState::kFinalized);

  device_manager::DeviceEvent event;
  event.level = device_manager::EventLevel::kError;
  event.target_state = device_manager::LifecycleState::kUnconfigured;
  requests = hook(hook_context(device_manager::LifecycleState::kActive, event, 1U));
  ASSERT_EQ(requests.size(), 2U);
  EXPECT_EQ(requests[0].from_state, device_manager::LifecycleState::kActive);
  EXPECT_EQ(requests[0].target_state, device_manager::LifecycleState::kInactive);
  EXPECT_EQ(requests[1].from_state, device_manager::LifecycleState::kInactive);
  EXPECT_EQ(requests[1].target_state, device_manager::LifecycleState::kUnconfigured);

  requests = hook(hook_context(device_manager::LifecycleState::kActive, event, 1U));
  EXPECT_TRUE(requests.empty());

  // 运行期断线事件 target 为 Active，hook 不做生命周期迁移（驱动核内部自愈）
  device_manager::DeviceEvent disconnect_event;
  disconnect_event.level = device_manager::EventLevel::kError;
  disconnect_event.target_state = device_manager::LifecycleState::kActive;
  requests = hook(hook_context(device_manager::LifecycleState::kActive, disconnect_event, 2U));
  EXPECT_TRUE(requests.empty());
}

TEST(Ch020ImuRuntime, EmitsDisconnectAndOnlineEventsWithoutLeavingActive)
{
  FakeSerialTransport * transport = nullptr;
  Ch020ImuRuntime runtime(
    Ch020ImuConfig{},
    [&transport](const Ch020ImuConfig &) {
      return make_transport(transport);
    });
  EventRecorder recorder;
  runtime.set_event_handler(recorder.handler());

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
  ASSERT_NE(transport, nullptr);

  transport->read_error = true;
  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("ch020_imu.disconnected");}));
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kActive);

  transport->read_error = false;
  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("ch020_imu.online");}));
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kActive);
  EXPECT_GE(transport->open_count.load(), 2);

  const auto events = recorder.events();
  auto disconnect_it = std::find_if(
    events.begin(), events.end(),
    [](const device_manager::DeviceEvent & event) {
      return event.code == "ch020_imu.disconnected";
    });
  ASSERT_NE(disconnect_it, events.end());
  EXPECT_EQ(disconnect_it->level, device_manager::EventLevel::kError);
  EXPECT_EQ(disconnect_it->source, "ch020_imu_driver");
  EXPECT_EQ(disconnect_it->target_state, device_manager::LifecycleState::kActive);

  auto online_it = std::find_if(
    events.begin(), events.end(),
    [](const device_manager::DeviceEvent & event) {
      return event.code == "ch020_imu.online";
    });
  ASSERT_NE(online_it, events.end());
  EXPECT_EQ(online_it->level, device_manager::EventLevel::kOk);

  // 恢复后数据通路继续工作
  const std::uint64_t frames_before = runtime.snapshot().frame_count;
  transport->push_bytes(build_imusol_frame());
  ASSERT_TRUE(
    wait_until(
      [&runtime, frames_before]() {
        return runtime.snapshot().frame_count > frames_before;
      }));
  EXPECT_TRUE(runtime.snapshot().connected);

  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kActive,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
}

TEST(Ch020ImuRuntime, DetectsDataStallAndReconnects)
{
  FakeSerialTransport * transport = nullptr;
  Ch020ImuRuntime runtime(
    Ch020ImuConfig{},
    [&transport](const Ch020ImuConfig &) {
      return make_transport(transport);
    });
  EventRecorder recorder;
  runtime.set_event_handler(recorder.handler());

  device_manager::ParameterMap parameters;
  parameters.emplace("data_timeout_ms", std::int64_t {100});
  parameters.emplace("reconnect_backoff_ms", std::int64_t {50});

  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive,
      std::move(parameters))).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_NE(transport, nullptr);

  // 读不出任何帧（板载 UART 拔线只表现为无数据），超时后判定断线
  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("ch020_imu.disconnected");}));
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kActive);

  const auto events = recorder.events();
  auto disconnect_it = std::find_if(
    events.begin(), events.end(),
    [](const device_manager::DeviceEvent & event) {
      return event.code == "ch020_imu.disconnected";
    });
  ASSERT_NE(disconnect_it, events.end());
  EXPECT_NE(disconnect_it->message.find("超时"), std::string::npos);

  // 数据恢复后回到在线并继续出帧
  transport->push_bytes(build_imusol_frame());
  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("ch020_imu.online");}));
  ASSERT_TRUE(wait_until([&runtime]() {return runtime.snapshot().frame_count >= 1U;}));

  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kActive,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
}

}  // namespace
}  // namespace ch020_imu_driver
