#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "device_manager_core/device_manager.hpp"
#include "oradar_ros_driver/oradar_lidar_runtime.hpp"

namespace oradar_ros_driver
{
namespace
{

using namespace std::chrono_literals;

class FakeOradarClient final : public OradarLidarClient
{
public:
  ord_sdk::error_t open() override
  {
    ++open_count;
    opened = open_result == ord_sdk::no_error;
    return open_result;
  }

  bool is_open() const override {return opened.load();}

  void close() override {opened = false;}

  void set_timeout(int timeout_ms) override {timeout = timeout_ms;}

  ord_sdk::error_t track_connect() override {return track_result;}
  ord_sdk::error_t enable_measure() override {measure_enabled = true; return ord_sdk::no_error;}
  ord_sdk::error_t disable_measure() override {measure_enabled = false; return ord_sdk::no_error;}
  ord_sdk::error_t enable_data_stream() override {stream_enabled = true; return ord_sdk::no_error;}
  ord_sdk::error_t disable_data_stream() override {stream_enabled = false; return ord_sdk::no_error;}

  ord_sdk::error_t get_scan_speed(std::uint32_t & speed) override
  {
    speed = motor_speed;
    return ord_sdk::no_error;
  }

  ord_sdk::error_t set_scan_speed(std::uint32_t speed) override
  {
    motor_speed = speed;
    ++set_speed_count;
    return ord_sdk::no_error;
  }

  ord_sdk::error_t get_tail_filter_level(std::uint32_t & level) override
  {
    level = filter_level;
    return ord_sdk::no_error;
  }

  ord_sdk::error_t set_tail_filter_level(std::uint32_t level) override
  {
    filter_level = level;
    ++set_filter_count;
    return ord_sdk::no_error;
  }

  ord_sdk::error_t get_scan_direction(std::uint32_t & direction) override
  {
    direction = scan_direction;
    return ord_sdk::no_error;
  }

  ord_sdk::error_t set_scan_direction(std::uint32_t direction) override
  {
    scan_direction = direction;
    ++set_direction_count;
    return ord_sdk::no_error;
  }

  ord_sdk::error_t apply_configs() override
  {
    ++apply_count;
    return ord_sdk::no_error;
  }

  ord_sdk::error_t get_scan_frame_data(ord_sdk::ScanFrameData & frame) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (frames.empty()) {
      std::this_thread::sleep_for(5ms);
      return ord_sdk::timed_out;
    }
    frame = frames.front();
    frames.erase(frames.begin());
    return ord_sdk::no_error;
  }

  void push_frame(const ord_sdk::ScanFrameData & frame)
  {
    std::lock_guard<std::mutex> lock(mutex);
    frames.push_back(frame);
  }

  std::atomic<int> open_count {0};
  std::atomic<int> set_speed_count {0};
  std::atomic<int> set_filter_count {0};
  std::atomic<int> set_direction_count {0};
  std::atomic<int> apply_count {0};
  std::atomic<bool> measure_enabled {false};
  std::atomic<bool> stream_enabled {false};
  std::atomic<bool> opened {false};
  ord_sdk::error_t open_result {ord_sdk::no_error};
  ord_sdk::error_t track_result {ord_sdk::no_error};
  int timeout {0};
  std::uint32_t motor_speed {10};
  std::uint32_t filter_level {1};
  std::uint32_t scan_direction {1};

private:
  std::mutex mutex;
  std::vector<ord_sdk::ScanFrameData> frames;
};

device_manager::TransitionRequest transition(
  device_manager::LifecycleState from,
  device_manager::LifecycleState target,
  device_manager::ParameterMap parameters = {})
{
  return device_manager::TransitionRequest{from, target, std::move(parameters)};
}

bool wait_until(const std::function<bool()> & predicate, std::chrono::milliseconds timeout = 1s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

ord_sdk::ScanFrameData scan_frame()
{
  ord_sdk::ScanFrameData frame;
  frame.timestamp = 42;
  frame.layers.resize(1);
  frame.layers[0].ranges = {0, 500, 1000, 2000, 0};
  frame.layers[0].intensities = {0, 10, 20, 30, 0};
  return frame;
}

TEST(OradarLidarRuntime, UsesNetworkParametersAndAppliesDeviceSettings)
{
  OradarLidarConfig created_config;
  FakeOradarClient * client = nullptr;
  OradarLidarRuntime runtime(
    [&created_config, &client](const OradarLidarConfig & config) {
      created_config = config;
      auto fake = std::make_unique<FakeOradarClient>();
      client = fake.get();
      return fake;
    });

  runtime.initialize({"rear_lidar", "lidar", true, {}});
  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);

  const device_manager::ParameterMap parameters {
    {"device.interface.ip_address", std::string {"192.168.71.101"}},
    {"device.interface.name", std::string {"eth1"}},
    {"device.interface.udp_port", std::int64_t {2007}},
    {"lidar.scan_frequency", std::int64_t {15}},
    {"lidar.filter_size", std::int64_t {0}},
    {"lidar.motor_dir", std::int64_t {0}},
    {"lidar.scan_angle_min", -120.0},
    {"lidar.scan_angle_max", 120.0},
    {"lidar.range_min", 0.1},
    {"lidar.range_max", 20.0},
  };
  const auto outcome = runtime.request_transition(
    transition(
      device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive,
      parameters));

  ASSERT_EQ(outcome.result, device_manager::TransitionResult::kSuccess);
  ASSERT_NE(client, nullptr);
  EXPECT_EQ(created_config.ip_address, "192.168.71.101");
  EXPECT_EQ(created_config.network_interface, "eth1");
  EXPECT_EQ(created_config.udp_port, 2007);
  EXPECT_EQ(created_config.motor_speed, 15);
  EXPECT_EQ(created_config.filter_size, 0);
  EXPECT_EQ(created_config.motor_dir, 0);
  EXPECT_EQ(created_config.angle_min_deg, -120.0);
  EXPECT_EQ(created_config.angle_max_deg, 120.0);
  EXPECT_EQ(created_config.range_min_m, 0.1);
  EXPECT_EQ(created_config.range_max_m, 20.0);
  EXPECT_EQ(client->open_count.load(), 1);
  EXPECT_EQ(client->set_speed_count.load(), 1);
  EXPECT_EQ(client->set_filter_count.load(), 1);
  EXPECT_EQ(client->set_direction_count.load(), 1);
  EXPECT_EQ(client->apply_count.load(), 1);
}

TEST(OradarLidarRuntime, AcceptsExistingEthernetInterfaceParameterNames)
{
  OradarLidarConfig created_config;
  FakeOradarClient * client = nullptr;
  OradarLidarRuntime runtime(
    [&created_config, &client](const OradarLidarConfig & config) {
      created_config = config;
      auto fake = std::make_unique<FakeOradarClient>();
      client = fake.get();
      return fake;
    });

  runtime.initialize({"front_lidar", "lidar", true, {}});
  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);

  const device_manager::ParameterMap parameters {
    {"device.interface.ethernet_ip", std::string {"192.168.71.100"}},
    {"device.interface.ethernet_port", std::int64_t {2007}},
  };
  const auto outcome = runtime.request_transition(
    transition(
      device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive,
      parameters));

  ASSERT_EQ(outcome.result, device_manager::TransitionResult::kSuccess);
  ASSERT_NE(client, nullptr);
  EXPECT_EQ(created_config.ip_address, "192.168.71.100");
  EXPECT_EQ(created_config.udp_port, 2007);
}

TEST(OradarLidarRuntime, PublishesScanWithDeviceSpecificTopicDefaults)
{
  FakeOradarClient * client = nullptr;
  OradarLidarRuntime runtime(
    [&client](const OradarLidarConfig &) {
      auto fake = std::make_unique<FakeOradarClient>();
      client = fake.get();
      return fake;
    });

  std::optional<sensor_msgs::msg::LaserScan> received;
  runtime.set_scan_handler(
    [&received](const sensor_msgs::msg::LaserScan & scan) {received = scan;});
  runtime.initialize({"rear_lidar", "lidar", true, {}});
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
        {{"device.interface.ip_address", std::string {"192.168.71.101"}}})).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_NE(client, nullptr);
  client->push_frame(scan_frame());

  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);

  ASSERT_TRUE(wait_until([&received]() {return received.has_value();}));
  EXPECT_EQ(received->header.frame_id, "left_behind_laser_link");
  EXPECT_FLOAT_EQ(received->angle_min, static_cast<float>(-135.0 * M_PI / 180.0));
  EXPECT_FLOAT_EQ(received->angle_max, static_cast<float>(135.0 * M_PI / 180.0));
  ASSERT_EQ(received->ranges.size(), 5U);
  EXPECT_FLOAT_EQ(received->ranges[1], 1.0F);
  EXPECT_FLOAT_EQ(received->ranges[2], 2.0F);
  EXPECT_FLOAT_EQ(received->ranges[3], 4.0F);

  EXPECT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kActive,
      device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
}

TEST(OradarLidarRuntime, ConfigureFailureEmitsConnectFailedEvent)
{
  FakeOradarClient * client = nullptr;
  OradarLidarRuntime runtime(
    [&client](const OradarLidarConfig &) {
      auto fake = std::make_unique<FakeOradarClient>();
      fake->open_result = ord_sdk::timed_out;
      client = fake.get();
      return fake;
    });

  std::vector<device_manager::DeviceEvent> events;
  runtime.set_event_handler(
    [&events](device_manager::DeviceEvent event) {events.push_back(std::move(event));});
  runtime.initialize({"front_lidar", "lidar", true, {}});
  ASSERT_EQ(
    runtime.request_transition(
      transition(device_manager::LifecycleState::kFinalized,
      device_manager::LifecycleState::kUnconfigured)).result,
    device_manager::TransitionResult::kSuccess);

  const auto outcome = runtime.request_transition(
    transition(
      device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive,
      {{"device.interface.ip_address", std::string {"192.168.71.100"}}}));

  ASSERT_NE(client, nullptr);
  EXPECT_EQ(outcome.result, device_manager::TransitionResult::kError);
  ASSERT_EQ(events.size(), 1U);
  EXPECT_EQ(events.front().code, "oradar_lidar.connect_failed");
  EXPECT_EQ(events.front().source, "oradar_lidar_driver");
  EXPECT_EQ(events.front().target_state, device_manager::LifecycleState::kUnconfigured);
}

}  // namespace
}  // namespace oradar_ros_driver
