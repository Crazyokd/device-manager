#include <gtest/gtest.h>

#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "device_manager_core/device_manager.hpp"
#include "device_manager_ros/device_registration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace smit_ros_driver
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
  bool write_bytes(const std::vector<std::uint8_t> & bytes) const
  {
    return ::write(master_, bytes.data(), bytes.size()) == static_cast<ssize_t>(bytes.size());
  }

private:
  int master_ {-1};
  std::string slave_name_;
};

class TemporaryConfig
{
public:
  TemporaryConfig()
  : path_("/tmp/smit_runtime_plugin_" + std::to_string(::getpid()) + ".yaml")
  {
    std::ofstream output(path_);
    output <<
      "instances:\n"
      "  - device_id: plugin_lidar\n"
      "    device_type: lidar\n"
      "    runtime: smit_ros_driver/SmitLidarRuntime\n";
  }

  ~TemporaryConfig() {::unlink(path_.c_str());}
  const std::string & path() const {return path_;}

private:
  std::string path_;
};

std::uint8_t crc8(const std::uint8_t * data, std::size_t length)
{
  std::uint8_t crc = 0;
  for (std::size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) != 0U ? static_cast<std::uint8_t>((crc << 1) ^ 0x07U) :
        static_cast<std::uint8_t>(crc << 1);
    }
  }
  return crc;
}

void append_u16_be(std::vector<std::uint8_t> & output, std::uint16_t value)
{
  output.push_back(static_cast<std::uint8_t>(value >> 8));
  output.push_back(static_cast<std::uint8_t>(value & 0xFFU));
}

std::vector<std::uint8_t> frame(
  std::uint8_t message_id, const std::vector<std::uint8_t> & body)
{
  const auto payload_length = static_cast<std::uint16_t>(body.size() + 2U);
  std::vector<std::uint8_t> result {
    0xA5, 0x5A,
    static_cast<std::uint8_t>(payload_length >> 8),
    static_cast<std::uint8_t>(payload_length & 0xFFU),
    message_id};
  result.insert(result.end(), body.begin(), body.end());
  result.push_back(crc8(result.data() + 4, result.size() - 4));
  return result;
}

std::vector<std::uint8_t> scan_stream()
{
  std::vector<std::uint8_t> config(31, 0);
  config[24] = 3;
  config[27] = 1;
  auto result = frame(0x02, config);
  for (const float start_angle : {100.0F, 190.0F, 300.0F, 100.0F, 190.0F}) {
    std::vector<std::uint8_t> body;
    append_u16_be(body, static_cast<std::uint16_t>(std::lround(start_angle * 100.0F)));
    append_u16_be(body, 1800);
    for (int point = 0; point < 4; ++point) {
      body.push_back(10);
      append_u16_be(body, 400);
    }
    body.insert(body.end(), {0, 0, 0});
    const auto point_frame = frame(0x01, body);
    result.insert(result.end(), point_frame.begin(), point_frame.end());
  }
  return result;
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
  auto loader_node = std::make_shared<rclcpp::Node>("smit_plugin_loader_test");
  auto registrations = device_manager_ros::load_device_registrations(
    *loader_node, config.path(), 100ms,
    [&terminal](const std::string &) {
      return device_manager::ParameterMap {
      {"device.enable", true},
      {"device.interface.serial_port", terminal.slave_name()},
      {"device.interface.serial_baudrate", std::int64_t {921600}},
      {"lidar.topic", std::string {"/plugin_scan"}},
      {"lidar.frame_id", std::string {"plugin_lidar_frame"}},
      {"data_timeout_ms", std::int64_t {5000}},
      };
    });
  ASSERT_EQ(registrations.size(), 1U);

  device_manager::DeviceManager manager(std::move(registrations));
  manager.start();
  ASSERT_TRUE(wait_until([&manager]() {
      manager.tick();
      const auto device = manager.device("plugin_lidar");
      return device && device->state == device_manager::LifecycleState::kActive;
    }));

  auto observer = std::make_shared<rclcpp::Node>("smit_plugin_output_test");
  std::optional<sensor_msgs::msg::LaserScan> received;
  auto subscription = observer->create_subscription<sensor_msgs::msg::LaserScan>(
    "/plugin_scan", rclcpp::SensorDataQoS(),
    [&received](sensor_msgs::msg::LaserScan::ConstSharedPtr message) {received = *message;});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  ASSERT_TRUE(wait_until([&observer]() {return observer->count_publishers("/plugin_scan") > 0U;}));
  ASSERT_TRUE(terminal.write_bytes(scan_stream()));
  ASSERT_TRUE(wait_until([&executor, &received]() {
      executor.spin_some();
      return received.has_value();
    }));
  EXPECT_EQ(received->header.frame_id, "plugin_lidar_frame");
  EXPECT_FALSE(received->ranges.empty());

  manager.stop();
  (void)subscription;
}

}  // namespace
}  // namespace smit_ros_driver
