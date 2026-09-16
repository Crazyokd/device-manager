#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "device_manager_core/device_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tws_battery_driver_ros2/battery_register_blocks.hpp"
#include "tws_battery_driver_ros2/tws_battery_runtime.hpp"

namespace tws_battery_driver_ros2
{
namespace
{

class FakeBatteryTransport final : public BatteryTransport
{
public:
  explicit FakeBatteryTransport(bool * disconnect_flag = nullptr)
  : disconnect_flag_(disconnect_flag)
  {
  }

  bool connect(std::string & error_message) override
  {
    if (!connect_result) {
      error_message = "connect failed";
      connected = false;
      return false;
    }
    error_message.clear();
    connected = true;
    return true;
  }

  void disconnect() override
  {
    connected = false;
    if (disconnect_flag_) {
      *disconnect_flag_ = true;
    }
  }

  bool is_connected() const override
  {
    return connected;
  }

  bool read_holding_registers(
    uint8_t,
    uint16_t register_address,
    uint16_t,
    std::vector<uint8_t> & register_bytes,
    std::string & error_message) override
  {
    ++read_count;
    if (!read_result) {
      error_message = "read failed";
      return false;
    }
    if (register_address == kFastBlockAStart) {
      register_bytes = {
        0x02, 0x00, 0x00, 0x00, 0xB5, 0xCF, 0x00, 0x00,
        0x94, 0xFD, 0xFF, 0xFF, 0xFE, 0x0C, 0xF9, 0x0C,
        0x48, 0x00, 0x47, 0x00, 0x47, 0x00, 0x46, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x0F, 0x80, 0x10, 0x40,
      };
      return true;
    }
    if (register_address == kFastBlockBStart) {
      register_bytes = {
        0x42, 0x00, 0x64, 0x00, 0x51, 0x26,
        0x00, 0x00, 0x17, 0x00, 0x00, 0x00,
      };
      return true;
    }
    register_bytes.assign(36U, 0U);
    return true;
  }

  bool connect_result{true};
  bool read_result{true};
  bool connected{false};
  std::atomic<int> read_count {0};

private:
  bool * disconnect_flag_;
};

std::unique_ptr<BatteryTransport> make_transport(
  FakeBatteryTransport *& current,
  bool * disconnect_flag = nullptr)
{
  auto transport = std::make_unique<FakeBatteryTransport>(disconnect_flag);
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

TEST(TwsBatteryRuntime, UsesAuthoritativeDeviceParameters)
{
  BatteryDriverConfig applied;
  TwsBatteryRuntime runtime(
    BatteryDriverConfig{},
    [&applied](const BatteryDriverConfig & config) {
      applied = config;
      return std::make_unique<FakeBatteryTransport>();
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
        {{"device.interface.can_name", std::string{"can1"}},
          {"battery.can_id", std::int64_t{65}},
          {"battery.use_extended_frame", true},
          {"battery.device_address", std::int64_t{2}}})).result,
    device_manager::TransitionResult::kSuccess);

  EXPECT_EQ(applied.can_interface, "can1");
  EXPECT_EQ(applied.can_id, 65U);
  EXPECT_TRUE(applied.use_extended_frame);
  EXPECT_EQ(applied.device_address, 2U);
}

TEST(TwsBatteryRuntime, OwnsPollingAndRosOutputWhileActive)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  FakeBatteryTransport * transport = nullptr;
  BatteryDriverConfig config;
  config.poll_interval_ms = 10;
  TwsBatteryRuntime runtime(
    config,
    [&transport](const BatteryDriverConfig &) {return make_transport(transport);});
  runtime.initialize(
    device_manager::DeviceDefinition{"battery", "battery", true, {}});

  auto observer = std::make_shared<rclcpp::Node>("tws_runtime_output_test");
  std::atomic<int> messages {0};
  auto subscription = observer->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery/state", rclcpp::SensorDataQoS(),
    [&messages](sensor_msgs::msg::BatteryState::ConstSharedPtr) {++messages;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);

  ASSERT_EQ(
    runtime.request_transition(transition(
      device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    runtime.request_transition(transition(
      device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    runtime.request_transition(transition(
      device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);

  ASSERT_TRUE(wait_until([&executor, &messages]() {
      executor.spin_some();
      return messages.load() > 0;
    }));
  ASSERT_GT(transport->read_count.load(), 0);

  ASSERT_EQ(
    runtime.request_transition(transition(
      device_manager::LifecycleState::kActive,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  const int stopped_count = transport->read_count.load();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(transport->read_count.load(), stopped_count);

  (void)subscription;
  rclcpp::shutdown();
}

TEST(TwsBatteryRuntime, MaterializesDisabledDriverOnDemand)
{
  FakeBatteryTransport * transport = nullptr;
  int transport_count = 0;
  TwsBatteryRuntime runtime(
    BatteryDriverConfig{},
    [&transport, &transport_count](const BatteryDriverConfig &) {
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

TEST(TwsBatteryRuntime, ImplementsBoundedLifecycleTransitions)
{
  FakeBatteryTransport * transport = nullptr;
  bool disconnected = false;
  TwsBatteryRuntime runtime(
    BatteryDriverConfig{},
    [&transport, &disconnected](const BatteryDriverConfig &) {
      return make_transport(transport, &disconnected);
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
  EXPECT_TRUE(transport->connected);
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kInactive);

  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kActive);

  EXPECT_TRUE(runtime.poll_once());
  EXPECT_TRUE(runtime.snapshot().connected);
  EXPECT_FLOAT_EQ(runtime.snapshot().pack_voltage_v, 53.173F);
  EXPECT_EQ(runtime.snapshot().work_state, 2U);

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
  EXPECT_TRUE(disconnected);
  EXPECT_EQ(runtime.state(), device_manager::LifecycleState::kUnconfigured);
}

TEST(TwsBatteryRuntime, DeviceManagerAutoStartsEnabledRuntime)
{
  FakeBatteryTransport * transport = nullptr;
  auto runtime = std::make_unique<TwsBatteryRuntime>(
    BatteryDriverConfig{},
    [&transport](const BatteryDriverConfig &) {return make_transport(transport);});
  auto * runtime_ptr = runtime.get();
  EXPECT_EQ(runtime_ptr->state(), device_manager::LifecycleState::kFinalized);
  EXPECT_FALSE(runtime_ptr->materialized());
  std::vector<device_manager::DeviceRegistration> registrations;
  registrations.push_back(
    {device_manager::DeviceDefinition{"battery", "battery", true, {}}, std::move(runtime)});
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

TEST(TwsBatteryRuntime, EmitsErrorEventAfterConsecutivePollFailures)
{
  FakeBatteryTransport * transport = nullptr;
  TwsBatteryRuntime runtime(
    BatteryDriverConfig{},
    [&transport](const BatteryDriverConfig &) {
      return make_transport(transport);
    });
  std::vector<device_manager::DeviceEvent> events;
  runtime.set_event_handler(
    [&events](device_manager::DeviceEvent event) {
      events.push_back(std::move(event));
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
  ASSERT_TRUE(runtime.poll_once());
  events.clear();

  transport->read_result = false;

  EXPECT_FALSE(runtime.poll_once());
  EXPECT_TRUE(events.empty());

  EXPECT_FALSE(runtime.poll_once());
  EXPECT_TRUE(events.empty());

  EXPECT_FALSE(runtime.poll_once());
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.back().level, device_manager::EventLevel::kError);
  EXPECT_EQ(events.back().code, "tws_battery.poll_failed");
  EXPECT_EQ(events.back().message, "read failed");
  EXPECT_EQ(events.back().target_state, device_manager::LifecycleState::kUnconfigured);

  EXPECT_FALSE(runtime.poll_once());
  EXPECT_EQ(events.size(), 1U);
}

TEST(TwsBatteryRuntime, KeepsLastGoodSnapshotUntilConsecutivePollFailures)
{
  FakeBatteryTransport * transport = nullptr;
  TwsBatteryRuntime runtime(
    BatteryDriverConfig{},
    [&transport](const BatteryDriverConfig &) {
      return make_transport(transport);
    });
  std::vector<device_manager::DeviceEvent> events;
  runtime.set_event_handler(
    [&events](device_manager::DeviceEvent event) {
      events.push_back(std::move(event));
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

  ASSERT_TRUE(runtime.poll_once());
  ASSERT_TRUE(runtime.snapshot().connected);
  const auto good_snapshot = runtime.snapshot();
  events.clear();

  transport->read_result = false;

  EXPECT_FALSE(runtime.poll_once());
  EXPECT_TRUE(runtime.snapshot().connected);
  EXPECT_EQ(runtime.snapshot().last_error, "");
  EXPECT_FLOAT_EQ(runtime.snapshot().pack_voltage_v, good_snapshot.pack_voltage_v);
  EXPECT_TRUE(events.empty());

  EXPECT_FALSE(runtime.poll_once());
  EXPECT_TRUE(runtime.snapshot().connected);
  EXPECT_TRUE(events.empty());

  EXPECT_FALSE(runtime.poll_once());
  EXPECT_FALSE(runtime.snapshot().connected);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.back().level, device_manager::EventLevel::kError);
  EXPECT_EQ(events.back().code, "tws_battery.poll_failed");
  EXPECT_EQ(events.back().target_state, device_manager::LifecycleState::kUnconfigured);
}

TEST(TwsBatteryRuntime, RegistersOneHookForAutoStartAndRecovery)
{
  FakeBatteryTransport * transport = nullptr;
  TwsBatteryRuntime runtime(
    BatteryDriverConfig{},
    [&transport](const BatteryDriverConfig &) {
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
}

}  // namespace
}  // namespace tws_battery_driver_ros2
