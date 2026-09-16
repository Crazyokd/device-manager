#include <gtest/gtest.h>

#include <unistd.h>

#include <chrono>
#include <fstream>
#include <string>

#include "device_manager_ros/device_registration.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tws_battery_driver_ros2
{
namespace
{

TEST(TwsBatteryRuntimePlugin, LoadsProductionRuntime)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);
  const std::string path =
    "/tmp/tws_runtime_plugin_" + std::to_string(::getpid()) + ".yaml";
  {
    std::ofstream output(path);
    output <<
      "instances:\n"
      "  - device_id: plugin_battery\n"
      "    device_type: battery\n"
      "    runtime: tws_battery_driver_ros2/TwsBatteryRuntime\n";
  }

  auto node = std::make_shared<rclcpp::Node>("tws_plugin_loader_test");
  auto registrations = device_manager_ros::load_device_registrations(
    *node, path, std::chrono::milliseconds(100),
    [](const std::string &) {
      return device_manager::ParameterMap {{"device.enable", false}};
    });
  ASSERT_EQ(registrations.size(), 1U);
  ASSERT_NE(registrations.front().runtime, nullptr);
  EXPECT_EQ(
    registrations.front().runtime->state(),
    device_manager::LifecycleState::kFinalized);

  ::unlink(path.c_str());
  rclcpp::shutdown();
}

}  // namespace
}  // namespace tws_battery_driver_ros2
