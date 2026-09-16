#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include "smit_ros_driver/smit_lidar_runtime.hpp"

namespace smit_ros_driver
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
// 构造真实帧格式的字节序列（与协议测试同一套格式）
// ---------------------------------------------------------------------------

uint8_t crc8(const uint8_t * data, std::size_t len)
{
  uint8_t crc = 0x00;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) != 0U ? static_cast<uint8_t>((crc << 1) ^ 0x07U) :
        static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

void append_u16_be(std::vector<uint8_t> & out, uint16_t value)
{
  out.push_back(static_cast<uint8_t>((value >> 8) & 0xFFU));
  out.push_back(static_cast<uint8_t>(value & 0xFFU));
}

std::vector<uint8_t> build_frame(uint8_t msg_id, const std::vector<uint8_t> & body)
{
  const uint16_t payload_len = static_cast<uint16_t>(body.size() + 2);
  std::vector<uint8_t> frame = {
    0xA5, 0x5A,
    static_cast<uint8_t>((payload_len >> 8) & 0xFFU),
    static_cast<uint8_t>(payload_len & 0xFFU),
    msg_id};
  frame.insert(frame.end(), body.begin(), body.end());
  frame.push_back(crc8(frame.data() + 4, frame.size() - 4));
  return frame;
}

std::vector<uint8_t> build_config_frame()
{
  std::vector<uint8_t> body(31, 0);
  body[24] = 3;   // motor_code → 20Hz
  body[27] = 1;   // freq_code → 20000 points/s
  return build_frame(0x02, body);
}

std::vector<uint8_t> build_point_frame(float start_angle_deg, std::size_t point_count)
{
  std::vector<uint8_t> body;
  append_u16_be(body, static_cast<uint16_t>(std::lround(start_angle_deg * 100.0F)));
  append_u16_be(body, 1800);   // delta_angle = 0.18°
  for (std::size_t i = 0; i < point_count; ++i) {
    body.push_back(10);        // 强度
    append_u16_be(body, 400);  // 距离 1.0m
  }
  body.insert(body.end(), {0, 0, 0});  // 24bit 时间戳
  return build_frame(0x01, body);
}

/// 一帧配置包 + 跨过 180° 边界的两圈点云，用于触发一次整圈 scan 回调
std::vector<uint8_t> build_full_sweep_stream()
{
  std::vector<uint8_t> stream = build_config_frame();
  for (const float start : {100.0F, 190.0F, 300.0F, 100.0F, 190.0F}) {
    const auto frame = build_point_frame(start, 4);
    stream.insert(stream.end(), frame.begin(), frame.end());
  }
  return stream;
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

TEST(SmitLidarRuntime, UsesAuthoritativeDeviceParameters)
{
  SmitLidarConfig applied;
  SmitLidarRuntime runtime(
    SmitLidarConfig{},
    [&applied](const SmitLidarConfig & config) {
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
        {{"device.interface.serial_port", std::string{"/dev/ttyS8"}},
          {"device.interface.serial_baudrate", std::int64_t{921600}},
          {"lidar.scan_angle_min", -135.5},
          {"lidar.scan_angle_max", 135.5}})).result,
    device_manager::TransitionResult::kSuccess);

  EXPECT_EQ(applied.port, "/dev/ttyS8");
  EXPECT_EQ(applied.baudrate, 921600);
  EXPECT_DOUBLE_EQ(applied.angle_min_deg, -135.5);
  EXPECT_DOUBLE_EQ(applied.angle_max_deg, 135.5);
}

TEST(SmitLidarRuntime, MaterializesDisabledDriverOnDemand)
{
  FakeSerialTransport * transport = nullptr;
  int transport_count = 0;
  SmitLidarRuntime runtime(
    SmitLidarConfig{},
    [&transport, &transport_count](const SmitLidarConfig &) {
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

TEST(SmitLidarRuntime, ImplementsBoundedLifecycleTransitions)
{
  FakeSerialTransport * transport = nullptr;
  SmitLidarRuntime runtime(
    SmitLidarConfig{},
    [&transport](const SmitLidarConfig &) {
      return make_transport(transport);
    });

  std::atomic<int> scan_count {0};
  runtime.set_scan_handler(
    [&scan_count](const SmitScan &) {
      ++scan_count;
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

  transport->push_bytes(build_full_sweep_stream());
  ASSERT_TRUE(wait_until([&scan_count]() {return scan_count.load() == 1;}));
  EXPECT_TRUE(runtime.snapshot().connected);
  EXPECT_TRUE(runtime.snapshot().config_received);
  EXPECT_EQ(runtime.snapshot().frame_count, 6U);

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

TEST(SmitLidarRuntime, DeviceManagerAutoStartsEnabledRuntime)
{
  FakeSerialTransport * transport = nullptr;
  auto runtime = std::make_unique<SmitLidarRuntime>(
    SmitLidarConfig{},
    [&transport](const SmitLidarConfig &) {return make_transport(transport);});
  auto * runtime_ptr = runtime.get();
  EXPECT_EQ(runtime_ptr->state(), device_manager::LifecycleState::kFinalized);
  EXPECT_FALSE(runtime_ptr->materialized());
  std::vector<device_manager::DeviceRegistration> registrations;
  registrations.push_back(
    {device_manager::DeviceDefinition{"lidar", "lidar", true, {}}, std::move(runtime)});
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

TEST(SmitLidarRuntime, EmitsConnectFailedEventWhenOpenFails)
{
  FakeSerialTransport * transport = nullptr;
  SmitLidarRuntime runtime(
    SmitLidarConfig{},
    [&transport](const SmitLidarConfig &) {
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

  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("smit_lidar.connect_failed");}));
  const auto events = recorder.events();
  EXPECT_EQ(events.back().level, device_manager::EventLevel::kError);
  EXPECT_EQ(events.back().code, "smit_lidar.connect_failed");
  EXPECT_EQ(events.back().message, "open failed");
  EXPECT_EQ(events.back().source, "smit_lidar_driver");
  EXPECT_EQ(events.back().target_state, device_manager::LifecycleState::kUnconfigured);
}

TEST(SmitLidarRuntime, RegistersOneHookForAutoStartAndRecovery)
{
  FakeSerialTransport * transport = nullptr;
  SmitLidarRuntime runtime(
    SmitLidarConfig{},
    [&transport](const SmitLidarConfig &) {
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

TEST(SmitLidarRuntime, EmitsDisconnectAndOnlineEventsWithoutLeavingActive)
{
  FakeSerialTransport * transport = nullptr;
  SmitLidarRuntime runtime(
    SmitLidarConfig{},
    [&transport](const SmitLidarConfig &) {
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
  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("smit_lidar.disconnected");}));
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kActive);

  transport->read_error = false;
  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("smit_lidar.online");}));
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kActive);
  EXPECT_GE(transport->open_count.load(), 2);

  const auto events = recorder.events();
  auto disconnect_it = std::find_if(
    events.begin(), events.end(),
    [](const device_manager::DeviceEvent & event) {
      return event.code == "smit_lidar.disconnected";
    });
  ASSERT_NE(disconnect_it, events.end());
  EXPECT_EQ(disconnect_it->level, device_manager::EventLevel::kError);
  EXPECT_EQ(disconnect_it->source, "smit_lidar_driver");
  EXPECT_EQ(disconnect_it->target_state, device_manager::LifecycleState::kActive);

  auto online_it = std::find_if(
    events.begin(), events.end(),
    [](const device_manager::DeviceEvent & event) {
      return event.code == "smit_lidar.online";
    });
  ASSERT_NE(online_it, events.end());
  EXPECT_EQ(online_it->level, device_manager::EventLevel::kOk);

  // 恢复后数据通路继续工作
  std::atomic<int> scan_count {0};
  runtime.set_scan_handler(
    [&scan_count](const SmitScan &) {
      ++scan_count;
    });
  transport->push_bytes(build_full_sweep_stream());
  ASSERT_TRUE(wait_until([&scan_count]() {return scan_count.load() == 1;}));
  EXPECT_TRUE(runtime.snapshot().connected);

  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kActive,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
}

TEST(SmitLidarRuntime, DetectsDataStallAndReconnects)
{
  FakeSerialTransport * transport = nullptr;
  SmitLidarRuntime runtime(
    SmitLidarConfig{},
    [&transport](const SmitLidarConfig &) {
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
  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("smit_lidar.disconnected");}));
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kActive);

  const auto events = recorder.events();
  auto disconnect_it = std::find_if(
    events.begin(), events.end(),
    [](const device_manager::DeviceEvent & event) {
      return event.code == "smit_lidar.disconnected";
    });
  ASSERT_NE(disconnect_it, events.end());
  EXPECT_NE(disconnect_it->message.find("超时"), std::string::npos);

  // 数据恢复后回到在线并继续出 scan
  std::atomic<int> scan_count {0};
  runtime.set_scan_handler(
    [&scan_count](const SmitScan &) {
      ++scan_count;
    });
  transport->push_bytes(build_full_sweep_stream());
  ASSERT_TRUE(wait_until([&recorder]() {return recorder.contains("smit_lidar.online");}));
  ASSERT_TRUE(wait_until([&scan_count]() {return scan_count.load() >= 1;}));

  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kActive,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
}

}  // namespace
}  // namespace smit_ros_driver
