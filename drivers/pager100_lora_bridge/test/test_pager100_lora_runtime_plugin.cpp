#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <functional>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "device_manager_core/device_manager.hpp"
#include "device_manager_ros/device_registration.hpp"
#include "pager100_lora_bridge/serial_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace pager100_lora_bridge
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

  /// 从 master 侧读出一个完整数据帧（即驱动写入 slave 侧的内容）。
  std::optional<SerialFrame> read_frame(std::chrono::milliseconds timeout) const
  {
    SerialFrameDecoder decoder;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      struct pollfd pfd {master_, POLLIN, 0};
      const int ready = ::poll(&pfd, 1, 50);
      if (ready <= 0) {
        continue;
      }
      std::uint8_t buffer[128];
      const ssize_t count = ::read(master_, buffer, sizeof(buffer));
      if (count <= 0) {
        continue;
      }
      const auto frames = decoder.feed(buffer, static_cast<std::size_t>(count));
      if (!frames.empty()) {
        return frames.front();
      }
    }
    return std::nullopt;
  }

private:
  int master_ {-1};
  std::string slave_name_;
};

class TemporaryConfig
{
public:
  TemporaryConfig()
  : path_("/tmp/pager100_runtime_plugin_" + std::to_string(::getpid()) + ".yaml")
  {
    std::ofstream output(path_);
    output <<
      "instances:\n"
      "  - device_id: plugin_lora\n"
      "    device_type: pager\n"
      "    runtime: pager100_lora_bridge/Pager100LoraRuntime\n";
  }

  ~TemporaryConfig() {::unlink(path_.c_str());}
  const std::string & path() const {return path_;}

private:
  std::string path_;
};

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

TEST_F(RosFixture, LoadsProductionRuntimeAndBridgesBothDirections)
{
  PseudoTerminal terminal;
  ASSERT_TRUE(terminal.valid());
  TemporaryConfig config;
  auto runtime_node = std::make_shared<rclcpp::Node>("pager100_plugin_loader_test");

  auto registrations = device_manager_ros::load_device_registrations(
    *runtime_node, config.path(), 100ms,
    [&terminal](const std::string & device_id) {
      EXPECT_EQ(device_id, "plugin_lora");
      return device_manager::ParameterMap {
      {"device.enable", true},
      {"device.interface.serial_port", terminal.slave_name()},
      {"device.interface.serial_baudrate", std::int64_t {115200}},
      };
    });
  ASSERT_EQ(registrations.size(), 1U);
  ASSERT_NE(registrations.front().runtime, nullptr);
  EXPECT_EQ(
    registrations.front().runtime->state(),
    device_manager::LifecycleState::kFinalized);

  device_manager::DeviceManager manager(std::move(registrations));
  manager.start();
  ASSERT_TRUE(
    wait_until(
      [&manager]() {
        manager.tick();
        const auto observation = manager.device("plugin_lora");
        return observation && observation->state == device_manager::LifecycleState::kActive;
      }));

  auto observer = std::make_shared<rclcpp::Node>("pager100_plugin_output_test");
  std::optional<std_msgs::msg::UInt8MultiArray> received;
  auto subscription = observer->create_subscription<std_msgs::msg::UInt8MultiArray>(
    "/pager100/lora/rx", 10,
    [&received](const std_msgs::msg::UInt8MultiArray & message) {received = message;});
  auto tx_publisher = observer->create_publisher<std_msgs::msg::UInt8MultiArray>(
    "/pager100/lora/tx", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  ASSERT_TRUE(
    wait_until([&observer]() {return observer->count_publishers("/pager100/lora/rx") > 0U;}));

  // 上行：串口帧 → /pager100/lora/rx
  const std::vector<std::uint8_t> uplink {0x10, 0x20, 0x30};
  ASSERT_TRUE(terminal.write_frame(encode_frame(uplink, 9)));
  ASSERT_TRUE(
    wait_until(
      [&executor, &received]() {
        executor.spin_some();
        return received.has_value();
      }));
  EXPECT_EQ(received->data, uplink);

  // 下行：/pager100/lora/tx → 串口帧
  ASSERT_TRUE(
    wait_until(
      [&observer]() {return observer->count_subscribers("/pager100/lora/tx") > 0U;}));
  std_msgs::msg::UInt8MultiArray downlink;
  downlink.data = {0x55, 0x66};
  tx_publisher->publish(downlink);
  const auto frame = terminal.read_frame(3s);
  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->frame_type, kFrameTypeData);
  EXPECT_EQ(frame->payload, downlink.data);

  ASSERT_TRUE(
    manager.enqueue(
      "plugin_lora",
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
        const auto observation = manager.device("plugin_lora");
        return observation && observation->state == device_manager::LifecycleState::kFinalized;
      }));

  manager.stop();
  (void)subscription;
}

}  // namespace
}  // namespace pager100_lora_bridge
