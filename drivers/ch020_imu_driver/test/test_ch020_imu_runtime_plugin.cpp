#include <gtest/gtest.h>

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "device_manager_core/device_manager.hpp"
#include "device_manager_ros/device_registration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace ch020_imu_driver
{
namespace
{

using namespace std::chrono_literals;

class RosFixture : public testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class PseudoTerminal
{
public:
  PseudoTerminal()
  : master_(::posix_openpt(O_RDWR | O_NOCTTY))
  {
    if (master_ >= 0 && ::grantpt(master_) == 0 && ::unlockpt(master_) == 0) {
      const char * name = ::ptsname(master_);
      if (name != nullptr) {
        slave_name_ = name;
      }
    }
  }

  ~PseudoTerminal()
  {
    if (master_ >= 0) {
      ::close(master_);
    }
  }

  bool valid() const {return master_ >= 0 && !slave_name_.empty();}
  const std::string & slave_name() const {return slave_name_;}

  bool write_frame(const std::vector<std::uint8_t> & frame) const
  {
    return ::write(master_, frame.data(), frame.size()) == static_cast<ssize_t>(frame.size());
  }

private:
  int master_ {-1};
  std::string slave_name_;
};

class TemporaryConfig
{
public:
  TemporaryConfig()
  : path_("/tmp/ch020_runtime_plugin_" + std::to_string(::getpid()) + ".yaml")
  {
    std::ofstream output(path_);
    output <<
      "instances:\n"
      "  - device_id: plugin_imu\n"
      "    device_type: imu\n"
      "    runtime: ch020_imu_driver/Ch020ImuRuntime\n";
  }

  ~TemporaryConfig() {::unlink(path_.c_str());}
  const std::string & path() const {return path_;}

private:
  std::string path_;
};

std::uint16_t crc16(const std::uint8_t * data, std::size_t length)
{
  std::uint16_t crc = 0;
  for (std::size_t index = 0; index < length; ++index) {
    crc ^= static_cast<std::uint16_t>(data[index]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<std::uint16_t>((crc << 1) ^ 0x1021U) :
        static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::vector<std::uint8_t> build_imusol_frame()
{
  std::vector<std::uint8_t> payload(76, 0);
  payload[0] = 0x91;
  const float acc_z = 1.0F;
  const float quat_w = 1.0F;
  std::memcpy(payload.data() + 20, &acc_z, sizeof(acc_z));
  std::memcpy(payload.data() + 60, &quat_w, sizeof(quat_w));

  const auto low = static_cast<std::uint8_t>(payload.size() & 0xFFU);
  const auto high = static_cast<std::uint8_t>((payload.size() >> 8) & 0xFFU);
  std::vector<std::uint8_t> crc_input {0x5A, 0xA5, low, high};
  crc_input.insert(crc_input.end(), payload.begin(), payload.end());
  const auto crc = crc16(crc_input.data(), crc_input.size());

  std::vector<std::uint8_t> frame {
    0x5A, 0xA5, low, high,
    static_cast<std::uint8_t>(crc & 0xFFU),
    static_cast<std::uint8_t>(crc >> 8)};
  frame.insert(frame.end(), payload.begin(), payload.end());
  return frame;
}

bool wait_until(const std::function<bool()> & predicate, std::chrono::seconds timeout = 3s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}

TEST_F(RosFixture, LoadsProductionRuntimeAndPublishesAfterFrameworkLifecycle)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  TemporaryConfig config;
  auto runtime_node = std::make_shared<rclcpp::Node>("ch020_plugin_loader_test");

  auto registrations = device_manager_ros::load_device_registrations(
    *runtime_node, config.path(), 100ms,
    [&terminal](const std::string & device_id) {
      EXPECT_EQ(device_id, "plugin_imu");
      return device_manager::ParameterMap {
      {"device.enable", true},
      {"device.interface.serial_port", terminal.slave_name()},
      {"device.interface.serial_baudrate", std::int64_t {115200}},
      {"imu.frame_id", std::string {"plugin_imu_link"}},
      {"data_timeout_ms", std::int64_t {5000}},
      };
    });
  ASSERT_EQ(registrations.size(), 1U);
  ASSERT_NE(registrations.front().runtime, nullptr);
  EXPECT_EQ(
    registrations.front().runtime->state(),
    device_manager::LifecycleState::kFinalized);
  EXPECT_FALSE(registrations.front().runtime->materialized());

  device_manager::DeviceManager manager(std::move(registrations));
  manager.start();
  ASSERT_TRUE(
    wait_until(
      [&manager]() {
        manager.tick();
        const auto observation = manager.device("plugin_imu");
        return observation && observation->state == device_manager::LifecycleState::kActive;
      }));

  auto observer = std::make_shared<rclcpp::Node>("ch020_plugin_output_test");
  std::optional<sensor_msgs::msg::Imu> received;
  auto subscription = observer->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10,
    [&received](sensor_msgs::msg::Imu::ConstSharedPtr message) {received = *message;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  ASSERT_TRUE(wait_until([&observer]() {return observer->count_publishers("/imu/data") > 0U;}));

  ASSERT_TRUE(terminal.write_frame(build_imusol_frame()));
  ASSERT_TRUE(
    wait_until(
      [&executor, &received]() {
        executor.spin_some();
        return received.has_value();
      }));
  EXPECT_EQ(received->header.frame_id, "plugin_imu_link");
  EXPECT_NEAR(received->linear_acceleration.z, 9.80665, 1e-5);

  ASSERT_TRUE(
    manager.enqueue(
      "plugin_imu",
      {
        {device_manager::LifecycleState::kActive,
          device_manager::LifecycleState::kInactive, {}},
        {device_manager::LifecycleState::kInactive,
          device_manager::LifecycleState::kUnconfigured, {}},
        {device_manager::LifecycleState::kUnconfigured,
          device_manager::LifecycleState::kFinalized, {}},
      },
      device_manager::RequestPriority::kUrgent).accepted);
  ASSERT_TRUE(
    wait_until(
      [&manager]() {
        const auto observation = manager.device("plugin_imu");
        return observation && observation->state == device_manager::LifecycleState::kFinalized;
      }));

  manager.stop();
  (void)subscription;
}

TEST_F(RosFixture, RepeatedActiveTeardownStopsDriverBeforeRosOutput)
{
  const auto frame = build_imusol_frame();
  auto runtime_node = std::make_shared<rclcpp::Node>("ch020_plugin_teardown_test");

  for (int iteration = 0; iteration < 10; ++iteration) {
    PseudoTerminal terminal;
    ASSERT_TRUE(terminal.valid());
    TemporaryConfig config;
    auto registrations = device_manager_ros::load_device_registrations(
      *runtime_node, config.path(), 100ms,
      [&terminal](const std::string &) {
        return device_manager::ParameterMap {
        {"device.enable", true},
        {"device.interface.serial_port", terminal.slave_name()},
        {"device.interface.serial_baudrate", std::int64_t {115200}},
        {"data_timeout_ms", std::int64_t {5000}},
        };
      });
    ASSERT_EQ(registrations.size(), 1U);

    std::atomic<bool> writing {true};
    std::thread writer;
    {
      device_manager::DeviceManager manager(std::move(registrations));
      manager.start();
      ASSERT_TRUE(
        wait_until(
          [&manager]() {
            manager.tick();
            const auto observation = manager.device("plugin_imu");
            return observation && observation->state == device_manager::LifecycleState::kActive;
          }));
      writer = std::thread(
        [&terminal, &frame, &writing]() {
          while (writing.load()) {
            (void)terminal.write_frame(frame);
            std::this_thread::sleep_for(1ms);
          }
        });
      std::this_thread::sleep_for(20ms);
      manager.stop();
    }
    writing.store(false);
    writer.join();
  }
}

}  // namespace
}  // namespace ch020_imu_driver
