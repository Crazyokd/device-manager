#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <string>

#include <unistd.h>

#include "device_manager_core/device_manager.hpp"
#include "device_manager_ros/device_registration.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gmsl_v4l2_camera_driver
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

class TemporaryConfig
{
public:
  TemporaryConfig()
  : path_("/tmp/gmsl_runtime_plugin_" + std::to_string(::getpid()) + ".yaml")
  {
    std::ofstream output(path_);
    output <<
      "instances:\n"
      "  - device_id: gmsl_cam\n"
      "    device_type: camera\n"
      "    runtime: gmsl_v4l2_camera_driver/GmslV4l2CameraRuntime\n";
  }

  ~TemporaryConfig() {::unlink(path_.c_str());}
  const std::string & path() const {return path_;}

private:
  std::string path_;
};

TEST_F(RosFixture, LoadsProductionRuntimePlugin)
{
  TemporaryConfig config;
  auto loader_node = std::make_shared<rclcpp::Node>("gmsl_plugin_loader_test");
  auto registrations = device_manager_ros::load_device_registrations(
    *loader_node, config.path(), 100ms,
    [](const std::string & device_id) {
      EXPECT_EQ(device_id, "gmsl_cam");
      return device_manager::ParameterMap{
      {"device.enable", false},
      {"device.interface.gmsl_device", std::string{"/dev/video0"}},
      {"camera.basic.camera_name", std::string{"gmsl_cam"}},
      {"camera.driver.image_resolution_width", std::int64_t{1280}},
      {"camera.driver.image_resolution_height", std::int64_t{720}},
      {"camera.driver.color_format", std::string{"NV12"}},
      {"camera.driver.color_fps", std::int64_t{30}},
      };
    });
  ASSERT_EQ(registrations.size(), 1U);
  ASSERT_NE(registrations.front().runtime, nullptr);
  EXPECT_EQ(
    registrations.front().runtime->state(),
    device_manager::LifecycleState::kFinalized);
}

}  // namespace
}  // namespace gmsl_v4l2_camera_driver
