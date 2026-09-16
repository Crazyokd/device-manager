#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstring>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <linux/videodev2.h>

#include "device_manager_core/device_manager.hpp"
#include "gmsl_v4l2_camera_driver/gmsl_v4l2_camera_runtime.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace gmsl_v4l2_camera_driver
{
namespace
{

using namespace std::chrono_literals;

class FakeCaptureBackend final : public CaptureBackend
{
public:
  bool configure(const GmslV4l2CameraConfig & config, std::string & error_message) override
  {
    applied_config = config;
    configured = configure_result;
    error_message = configured ? "" : "open failed";
    return configured;
  }

  std::optional<CapturedFrame> read_frame(
    std::chrono::milliseconds,
    std::string & error_message) override
  {
    if (!configured) {
      error_message = "not configured";
      return std::nullopt;
    }
    if (!frames.empty()) {
      auto frame = std::move(frames.front());
      frames.erase(frames.begin());
      error_message.clear();
      return frame;
    }
    std::this_thread::sleep_for(5ms);
    error_message = "capture timeout";
    return std::nullopt;
  }

  void close() override
  {
    configured = false;
    close_count += 1;
  }

  GmslV4l2CameraConfig applied_config;
  bool configure_result {true};
  bool configured {false};
  int close_count {0};
  std::vector<CapturedFrame> frames;
};

std::uint32_t test_crc32(const std::vector<std::uint8_t> & data, std::size_t begin, std::size_t end)
{
  std::uint32_t crc = 0xffffffffU;
  for (std::size_t index = begin; index < end; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc >> 1U) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
  }
  return crc ^ 0xffffffffU;
}

void write_u16(std::vector<std::uint8_t> & data, std::size_t offset, std::uint16_t value)
{
  data[offset] = static_cast<std::uint8_t>(value & 0xffU);
  data[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
}

void write_u32(std::vector<std::uint8_t> & data, std::size_t offset, std::uint32_t value)
{
  data[offset] = static_cast<std::uint8_t>(value & 0xffU);
  data[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xffU);
  data[offset + 2U] = static_cast<std::uint8_t>((value >> 16U) & 0xffU);
  data[offset + 3U] = static_cast<std::uint8_t>((value >> 24U) & 0xffU);
}

void write_double(std::vector<std::uint8_t> & data, std::size_t offset, double value)
{
  static_assert(sizeof(double) == 8U);
  std::uint8_t bytes[8];
  std::memcpy(bytes, &value, sizeof(bytes));
  for (std::size_t index = 0; index < sizeof(bytes); ++index) {
    data[offset + index] = bytes[index];
  }
}

std::vector<std::uint8_t> valid_ox01f10_otp()
{
  std::vector<std::uint8_t> data(0x2000U, 0xffU);
  write_u16(data, 0x60U, 1280U);
  write_u16(data, 0x62U, 960U);
  data[0x64U] = 0x02U;
  write_double(data, 0x65U, 316.4589527674);
  write_double(data, 0x6dU, 316.4021688498);
  write_double(data, 0x75U, 641.2050576828);
  write_double(data, 0x7dU, 483.1482381048);
  write_double(data, 0xc5U, 0.0975673817);
  write_double(data, 0xcdU, -0.0076456599);
  write_double(data, 0xd5U, -0.0015543389);
  write_double(data, 0xddU, 0.000051554);
  write_u32(data, 0x11cU, test_crc32(data, 0x60U, 0x11cU));

  const std::string serial_number = "OTP-SN-001";
  std::copy(serial_number.begin(), serial_number.end(), data.begin() + 0x120U);
  write_u32(data, 0x141U, test_crc32(data, 0x120U, 0x130U));
  return data;
}

class FakeOtpReader final : public OtpReader
{
public:
  OtpReadResult read(const GmslV4l2CameraConfig &) override
  {
    called = true;
    return result;
  }

  bool called {false};
  OtpReadResult result;
};

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

TEST(GmslV4l2CameraConfigTest, DefaultsCaptureRateToTenHertz)
{
  EXPECT_EQ(GmslV4l2CameraConfig{}.fps, 10);
  EXPECT_EQ(GmslV4l2CameraConfig{}.calibration_crop_y, -1);
}

device_manager::TransitionRequest transition(
  device_manager::LifecycleState from,
  device_manager::LifecycleState target,
  device_manager::ParameterMap parameters = {})
{
  return device_manager::TransitionRequest{from, target, std::move(parameters)};
}

device_manager::ParameterMap camera_parameters()
{
  return {
    {"device.enable", true},
    {"device.interface.gmsl_device", std::string{"/dev/video7"}},
    {"camera.basic.camera_name", std::string{"front_gmsl"}},
    {"camera.basic.frame_id", std::string{"front_gmsl_link"}},
    {"camera.basic.serial_number", std::string{"H190TA-I05252006"}},
    {"camera.driver.image_topic", std::string{"image_raw"}},
    {"camera.driver.serial_number_topic", std::string{"serial_number"}},
    {"camera.driver.read_otp_on_start", false},
    {"camera.driver.intrinsic_params",
      std::string{
        R"json({"width":2,"height":1,"distortion_model":"equidistant","fx":316.4589527674,"fy":316.4021688498,"cx":641.2050576828,"cy":483.1482381048,"d":[0.0975673817,-0.0076456599,-0.0015543389,0.000051554]})json"}},
    {"camera.driver.image_resolution_width", std::int64_t{2}},
    {"camera.driver.image_resolution_height", std::int64_t{1}},
    {"camera.driver.color_format", std::string{"BGR8"}},
    {"camera.driver.color_fps", std::int64_t{30}},
    {"camera.driver.undistort_focal_scale", 0.55},
  };
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

std::vector<std::uint8_t> opencv_fisheye_remap(
  const std::vector<std::uint8_t> & bgr_data,
  int width,
  int height,
  double fx,
  double fy,
  double cx,
  double cy,
  const std::array<double, 4> & d,
  double focal_scale)
{
  const cv::Mat input(height, width, CV_8UC3, const_cast<std::uint8_t *>(bgr_data.data()));
  const cv::Mat k = (cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
  const cv::Mat distortion =
    (cv::Mat_<double>(4, 1) << d[0], d[1], d[2], d[3]);
  const cv::Mat r = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat output_k = k.clone();
  output_k.at<double>(0, 0) *= focal_scale;
  output_k.at<double>(1, 1) *= focal_scale;
  cv::Mat map_x;
  cv::Mat map_y;
  cv::fisheye::initUndistortRectifyMap(
    k,
    distortion,
    r,
    output_k,
    cv::Size(width, height),
    CV_32FC1,
    map_x,
    map_y);
  cv::Mat output;
  cv::remap(input, output, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  std::vector<std::uint8_t> result(output.total() * output.elemSize());
  std::memcpy(result.data(), output.data, result.size());
  return result;
}

TEST_F(RosFixture, PublishesRvizImageAndCameraInfoFromDeviceParameters)
{
  FakeCaptureBackend * fake = nullptr;
  std::vector<device_manager::DeviceEvent> events;
  GmslV4l2CameraRuntime runtime(
    [&fake]() -> std::unique_ptr<CaptureBackend> {
      auto backend = std::make_unique<FakeCaptureBackend>();
      backend->frames.push_back(
        CapturedFrame{
          2,
          1,
          "BGR8",
          {0x10, 0x20, 0x30, 0x40, 0x50, 0x60}});
      fake = backend.get();
      return backend;
    });
  runtime.set_event_handler(
    [&events](device_manager::DeviceEvent event) {
      events.push_back(std::move(event));
    });

  auto parameters = camera_parameters();
  parameters["camera.driver.undistort_image"] = false;
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(fake->applied_config.video_device, "/dev/video7");
  EXPECT_EQ(fake->applied_config.camera_name, "front_gmsl");
  EXPECT_EQ(fake->applied_config.frame_id, "front_gmsl_link");

  auto observer = std::make_shared<rclcpp::Node>("gmsl_runtime_output_test");
  std::optional<sensor_msgs::msg::Image> image;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info;
  std::optional<std_msgs::msg::String> serial_number;
  auto image_sub = observer->create_subscription<sensor_msgs::msg::Image>(
    "/front_gmsl/image_raw", rclcpp::SensorDataQoS(),
    [&image](sensor_msgs::msg::Image::ConstSharedPtr message) {image = *message;});
  auto info_sub = observer->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/front_gmsl/camera_info", 10,
    [&camera_info](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      camera_info = *message;
    });
  auto serial_sub = observer->create_subscription<std_msgs::msg::String>(
    "/front_gmsl/serial_number", 10,
    [&serial_number](std_msgs::msg::String::ConstSharedPtr message) {
      serial_number = *message;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  ASSERT_TRUE(
    wait_until(
      [&observer]() {return observer->count_publishers("/front_gmsl/image_raw") > 0U;}));
  EXPECT_EQ(observer->count_publishers("/front_gmsl/image_raw_distorted"), 0U);

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(
    wait_until(
      [&executor, &image, &camera_info, &serial_number]() {
        executor.spin_some();
        return image.has_value() && camera_info.has_value() && serial_number.has_value();
      }));
  EXPECT_EQ(image->header.frame_id, "front_gmsl_link");
  EXPECT_EQ(image->height, 1U);
  EXPECT_EQ(image->width, 2U);
  EXPECT_EQ(image->encoding, "bgr8");
  EXPECT_EQ(image->step, 6U);
  EXPECT_EQ(image->data, (std::vector<std::uint8_t>{0x10, 0x20, 0x30, 0x40, 0x50, 0x60}));
  EXPECT_EQ(camera_info->header.frame_id, "front_gmsl_link");
  EXPECT_EQ(camera_info->height, 1U);
  EXPECT_EQ(camera_info->width, 2U);
  EXPECT_EQ(camera_info->distortion_model, "equidistant");
  EXPECT_EQ(
    camera_info->d,
    (std::vector<double>{0.0975673817, -0.0076456599, -0.0015543389, 0.000051554}));
  EXPECT_DOUBLE_EQ(camera_info->k[0], 316.4589527674);
  EXPECT_DOUBLE_EQ(camera_info->k[2], 641.2050576828);
  EXPECT_DOUBLE_EQ(camera_info->k[4], 316.4021688498);
  EXPECT_DOUBLE_EQ(camera_info->k[5], 483.1482381048);
  EXPECT_DOUBLE_EQ(camera_info->k[8], 1.0);
  EXPECT_DOUBLE_EQ(camera_info->r[0], 1.0);
  EXPECT_DOUBLE_EQ(camera_info->r[4], 1.0);
  EXPECT_DOUBLE_EQ(camera_info->r[8], 1.0);
  EXPECT_DOUBLE_EQ(camera_info->p[0], 316.4589527674);
  EXPECT_DOUBLE_EQ(camera_info->p[2], 641.2050576828);
  EXPECT_DOUBLE_EQ(camera_info->p[5], 316.4021688498);
  EXPECT_DOUBLE_EQ(camera_info->p[6], 483.1482381048);
  EXPECT_EQ(serial_number->data, "H190TA-I05252006");
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.front().values.at("serial_number"), "H190TA-I05252006");

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kActive,
        device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  (void)image_sub;
  (void)info_sub;
  (void)serial_sub;
}

TEST_F(RosFixture, RotatesImageAndCameraPrincipalPointTogether)
{
  FakeCaptureBackend * fake = nullptr;
  GmslV4l2CameraRuntime runtime(
    [&fake]() -> std::unique_ptr<CaptureBackend> {
      auto backend = std::make_unique<FakeCaptureBackend>();
      backend->frames.push_back(
        CapturedFrame{2, 1, "BGR8", {0x10, 0x20, 0x30, 0x40, 0x50, 0x60}});
      fake = backend.get();
      return backend;
    });

  auto parameters = camera_parameters();
  parameters["camera.driver.undistort_image"] = false;
  parameters["camera.driver.rotate_180"] = true;
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_TRUE(fake->applied_config.rotate_180);

  auto observer = std::make_shared<rclcpp::Node>("gmsl_runtime_rotate_output_test");
  std::optional<sensor_msgs::msg::Image> image;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info;
  auto image_sub = observer->create_subscription<sensor_msgs::msg::Image>(
    "/front_gmsl/image_raw", rclcpp::SensorDataQoS(),
    [&image](sensor_msgs::msg::Image::ConstSharedPtr message) {image = *message;});
  auto info_sub = observer->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/front_gmsl/camera_info", 10,
    [&camera_info](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      camera_info = *message;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  ASSERT_TRUE(wait_until(
      [&observer]() {return observer->count_publishers("/front_gmsl/image_raw") > 0U;}));

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(wait_until(
      [&executor, &image, &camera_info]() {
        executor.spin_some();
        return image.has_value() && camera_info.has_value();
      }));
  EXPECT_EQ(image->data, (std::vector<std::uint8_t>{0x40, 0x50, 0x60, 0x10, 0x20, 0x30}));
  EXPECT_DOUBLE_EQ(camera_info->k[2], 1.0 - 641.2050576828);
  EXPECT_DOUBLE_EQ(camera_info->k[5], -483.1482381048);
  EXPECT_DOUBLE_EQ(camera_info->p[2], camera_info->k[2]);
  EXPECT_DOUBLE_EQ(camera_info->p[6], camera_info->k[5]);

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kActive,
        device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  (void)image_sub;
  (void)info_sub;
}

TEST_F(RosFixture, PublishesUndistortedImageAndConfiguredDistortedImageTopic)
{
  FakeCaptureBackend * fake = nullptr;
  const std::vector<std::uint8_t> distorted_data = {
    0, 1, 2, 3, 4, 5, 6, 7, 8,
    9, 10, 11, 12, 13, 14, 15, 16, 17,
    18, 19, 20, 21, 22, 23, 24, 25, 26};
  GmslV4l2CameraRuntime runtime(
    [&fake, &distorted_data]() -> std::unique_ptr<CaptureBackend> {
      auto backend = std::make_unique<FakeCaptureBackend>();
      backend->frames.push_back(CapturedFrame{3, 3, "BGR8", distorted_data});
      fake = backend.get();
      return backend;
    });

  auto parameters = camera_parameters();
  parameters["camera.driver.intrinsic_params"] = std::string{
    R"json({"width":3,"height":3,"distortion_model":"equidistant","fx":1.0,"fy":1.0,"cx":1.0,"cy":1.0,"d":[0.3,0.0,0.0,0.0]})json"};
  parameters["camera.driver.image_resolution_width"] = std::int64_t{3};
  parameters["camera.driver.image_resolution_height"] = std::int64_t{3};
  parameters["camera.driver.publish_distorted_image"] = true;
  parameters["camera.driver.distorted_image_topic"] = std::string{"image_raw_distorted"};

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);

  auto observer = std::make_shared<rclcpp::Node>("gmsl_runtime_undistorted_output_test");
  std::optional<sensor_msgs::msg::Image> image;
  std::optional<sensor_msgs::msg::Image> distorted_image;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info;
  auto image_sub = observer->create_subscription<sensor_msgs::msg::Image>(
    "/front_gmsl/image_raw", rclcpp::SensorDataQoS(),
    [&image](sensor_msgs::msg::Image::ConstSharedPtr message) {image = *message;});
  auto distorted_image_sub = observer->create_subscription<sensor_msgs::msg::Image>(
    "/front_gmsl/image_raw_distorted", rclcpp::SensorDataQoS(),
    [&distorted_image](sensor_msgs::msg::Image::ConstSharedPtr message) {
      distorted_image = *message;
    });
  auto info_sub = observer->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/front_gmsl/camera_info", 10,
    [&camera_info](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      camera_info = *message;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  ASSERT_TRUE(
    wait_until(
      [&observer]() {return observer->count_publishers("/front_gmsl/image_raw") > 0U;}));
  ASSERT_TRUE(
    wait_until(
      [&observer]() {
        return observer->count_publishers("/front_gmsl/image_raw_distorted") > 0U;
      }));

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(
    wait_until(
      [&executor, &image, &distorted_image, &camera_info]() {
        executor.spin_some();
        return image.has_value() && distorted_image.has_value() && camera_info.has_value();
      }));

  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(image->height, 3U);
  EXPECT_EQ(image->width, 3U);
  EXPECT_NE(image->data, distorted_data);
  ASSERT_TRUE(distorted_image.has_value());
  EXPECT_EQ(distorted_image->height, 3U);
  EXPECT_EQ(distorted_image->width, 3U);
  EXPECT_EQ(distorted_image->data, distorted_data);
  ASSERT_TRUE(camera_info.has_value());
  EXPECT_EQ(camera_info->distortion_model, "plumb_bob");
  EXPECT_EQ(camera_info->d, (std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0}));

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kActive,
        device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  (void)image_sub;
  (void)distorted_image_sub;
  (void)info_sub;
}

TEST_F(RosFixture, PublishesUndistortedImageAtCaptureResolutionWithWideFov)
{
  FakeCaptureBackend * fake = nullptr;
  const std::vector<std::uint8_t> distorted_data = {
    0, 16, 32, 48, 64, 80};
  GmslV4l2CameraRuntime runtime(
    [&fake, &distorted_data]() -> std::unique_ptr<CaptureBackend> {
      auto backend = std::make_unique<FakeCaptureBackend>();
      backend->frames.push_back(CapturedFrame{2, 1, "BGR8", distorted_data});
      fake = backend.get();
      return backend;
    });

  auto parameters = camera_parameters();
  parameters["camera.driver.intrinsic_params"] = std::string{
    R"json({"width":4,"height":2,"distortion_model":"equidistant","fx":2.0,"fy":2.0,"cx":2.0,"cy":1.0,"d":[0.0,0.0,0.0,0.0]})json"};
  parameters["camera.driver.image_resolution_width"] = std::int64_t{2};
  parameters["camera.driver.image_resolution_height"] = std::int64_t{1};
  parameters["camera.driver.publish_distorted_image"] = true;

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);

  auto observer = std::make_shared<rclcpp::Node>("gmsl_runtime_calibration_resolution_test");
  std::optional<sensor_msgs::msg::Image> image;
  std::optional<sensor_msgs::msg::Image> distorted_image;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info;
  auto image_sub = observer->create_subscription<sensor_msgs::msg::Image>(
    "/front_gmsl/image_raw", rclcpp::SensorDataQoS(),
    [&image](sensor_msgs::msg::Image::ConstSharedPtr message) {image = *message;});
  auto distorted_image_sub = observer->create_subscription<sensor_msgs::msg::Image>(
    "/front_gmsl/image_raw_distorted", rclcpp::SensorDataQoS(),
    [&distorted_image](sensor_msgs::msg::Image::ConstSharedPtr message) {
      distorted_image = *message;
    });
  auto info_sub = observer->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/front_gmsl/camera_info", 10,
    [&camera_info](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      camera_info = *message;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  ASSERT_TRUE(
    wait_until(
      [&observer]() {return observer->count_publishers("/front_gmsl/image_raw") > 0U;}));

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(
    wait_until(
      [&executor, &image, &distorted_image, &camera_info]() {
        executor.spin_some();
        return image.has_value() && distorted_image.has_value() && camera_info.has_value();
      }));

  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(image->height, 1U);
  EXPECT_EQ(image->width, 2U);
  EXPECT_EQ(image->step, 6U);
  EXPECT_EQ(image->data.size(), 6U);
  ASSERT_TRUE(distorted_image.has_value());
  EXPECT_EQ(distorted_image->height, 1U);
  EXPECT_EQ(distorted_image->width, 2U);
  EXPECT_EQ(distorted_image->step, 6U);
  EXPECT_EQ(distorted_image->data, distorted_data);
  ASSERT_TRUE(camera_info.has_value());
  EXPECT_EQ(camera_info->height, 1U);
  EXPECT_EQ(camera_info->width, 2U);
  EXPECT_LT(camera_info->k[0], 1.0);
  EXPECT_LT(camera_info->k[4], 1.0);
  EXPECT_EQ(camera_info->distortion_model, "plumb_bob");
  EXPECT_EQ(camera_info->d, (std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0}));

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kActive,
        device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  (void)image_sub;
  (void)distorted_image_sub;
  (void)info_sub;
}

TEST_F(RosFixture, UsesConfiguredCalibrationCropWhenCaptureHeightIsSmaller)
{
  FakeCaptureBackend * fake = nullptr;
  std::vector<std::uint8_t> distorted_data(4U * 2U * 3U);
  for (std::size_t index = 0; index < distorted_data.size(); ++index) {
    distorted_data[index] = static_cast<std::uint8_t>((index * 7U) % 253U);
  }
  GmslV4l2CameraRuntime runtime(
    [&fake, &distorted_data]() -> std::unique_ptr<CaptureBackend> {
      auto backend = std::make_unique<FakeCaptureBackend>();
      backend->frames.push_back(CapturedFrame{4, 2, "BGR8", distorted_data});
      fake = backend.get();
      return backend;
    });

  constexpr double kFx = 2.4;
  constexpr double kFy = 2.7;
  constexpr double kCx = 1.8;
  constexpr double kCy = 1.6;
  constexpr double kCropY = 0.0;
  constexpr double kFocalScale = 0.55;
  constexpr std::array<double, 4> kDistortion {0.2, -0.03, 0.01, -0.001};

  auto parameters = camera_parameters();
  parameters["camera.driver.intrinsic_params"] = std::string{
    R"json({"width":4,"height":3,"type":"fisheye","fx":2.4,"fy":2.7,"cx":1.8,"cy":1.6,"d":[0.2,-0.03,0.01,-0.001]})json"};
  parameters["camera.driver.image_resolution_width"] = std::int64_t{4};
  parameters["camera.driver.image_resolution_height"] = std::int64_t{2};
  parameters["camera.driver.calibration_crop_y"] = std::int64_t{0};

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);

  auto observer = std::make_shared<rclcpp::Node>("gmsl_runtime_centered_crop_test");
  std::optional<sensor_msgs::msg::Image> image;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info;
  auto image_sub = observer->create_subscription<sensor_msgs::msg::Image>(
    "/front_gmsl/image_raw", rclcpp::SensorDataQoS(),
    [&image](sensor_msgs::msg::Image::ConstSharedPtr message) {image = *message;});
  auto info_sub = observer->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/front_gmsl/camera_info", 10,
    [&camera_info](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      camera_info = *message;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  ASSERT_TRUE(
    wait_until(
      [&observer]() {return observer->count_publishers("/front_gmsl/image_raw") > 0U;}));

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(
    wait_until(
      [&executor, &image, &camera_info]() {
        executor.spin_some();
        return image.has_value() && camera_info.has_value();
      }));

  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(
    image->data,
    opencv_fisheye_remap(
      distorted_data,
      4,
      2,
      kFx,
      kFy,
      kCx,
      kCy - kCropY,
      kDistortion,
      kFocalScale));
  ASSERT_TRUE(camera_info.has_value());
  EXPECT_DOUBLE_EQ(camera_info->k[0], kFx * kFocalScale);
  EXPECT_DOUBLE_EQ(camera_info->k[2], kCx);
  EXPECT_DOUBLE_EQ(camera_info->k[4], kFy * kFocalScale);
  EXPECT_DOUBLE_EQ(camera_info->k[5], kCy - kCropY);
  EXPECT_EQ(camera_info->distortion_model, "plumb_bob");

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kActive,
        device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  (void)image_sub;
  (void)info_sub;
}

TEST_F(RosFixture, TreatsVendorFisheyeTypeAsOpenCvFisheyeModel)
{
  FakeCaptureBackend * fake = nullptr;
  std::vector<std::uint8_t> distorted_data(5U * 5U * 3U);
  for (std::size_t index = 0; index < distorted_data.size(); ++index) {
    distorted_data[index] = static_cast<std::uint8_t>((index * 11U) % 251U);
  }
  GmslV4l2CameraRuntime runtime(
    [&fake, &distorted_data]() -> std::unique_ptr<CaptureBackend> {
      auto backend = std::make_unique<FakeCaptureBackend>();
      backend->frames.push_back(CapturedFrame{5, 5, "BGR8", distorted_data});
      fake = backend.get();
      return backend;
    });

  constexpr double kFx = 2.0;
  constexpr double kFy = 2.1;
  constexpr double kCx = 2.0;
  constexpr double kCy = 2.0;
  constexpr double kFocalScale = 0.55;
  constexpr std::array<double, 4> kDistortion {0.4, -0.08, 0.02, -0.003};

  auto parameters = camera_parameters();
  parameters["camera.driver.intrinsic_params"] = std::string{
    R"json({"width":5,"height":5,"type":"fisheye","fx":2.0,"fy":2.1,"cx":2.0,"cy":2.0,"d":[0.4,-0.08,0.02,-0.003]})json"};
  parameters["camera.driver.image_resolution_width"] = std::int64_t{5};
  parameters["camera.driver.image_resolution_height"] = std::int64_t{5};

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_NE(fake, nullptr);
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);

  auto observer = std::make_shared<rclcpp::Node>("gmsl_runtime_vendor_fisheye_type_test");
  std::optional<sensor_msgs::msg::Image> image;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info;
  auto image_sub = observer->create_subscription<sensor_msgs::msg::Image>(
    "/front_gmsl/image_raw", rclcpp::SensorDataQoS(),
    [&image](sensor_msgs::msg::Image::ConstSharedPtr message) {image = *message;});
  auto info_sub = observer->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/front_gmsl/camera_info", 10,
    [&camera_info](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      camera_info = *message;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  ASSERT_TRUE(
    wait_until(
      [&observer]() {return observer->count_publishers("/front_gmsl/image_raw") > 0U;}));

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(
    wait_until(
      [&executor, &image, &camera_info]() {
        executor.spin_some();
        return image.has_value() && camera_info.has_value();
      }));

  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(
    image->data,
    opencv_fisheye_remap(
      distorted_data,
      5,
      5,
      kFx,
      kFy,
      kCx,
      kCy,
      kDistortion,
      kFocalScale));
  ASSERT_TRUE(camera_info.has_value());
  EXPECT_EQ(camera_info->distortion_model, "plumb_bob");
  EXPECT_EQ(camera_info->d, (std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(camera_info->k[0], kFx * kFocalScale);
  EXPECT_DOUBLE_EQ(camera_info->k[4], kFy * kFocalScale);

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kActive,
        device_manager::LifecycleState::kInactive)).result,
    device_manager::TransitionResult::kSuccess);
  (void)image_sub;
  (void)info_sub;
}

TEST_F(RosFixture, ReadsOtpOnConfigureAndPrefersOtpCalibrationOverConfiguredParameters)
{
  FakeCaptureBackend * fake_capture = nullptr;
  FakeOtpReader * fake_otp = nullptr;
  std::vector<device_manager::DeviceEvent> events;
  GmslV4l2CameraRuntime runtime(
    [&fake_capture]() -> std::unique_ptr<CaptureBackend> {
      auto backend = std::make_unique<FakeCaptureBackend>();
      backend->frames.push_back(
        CapturedFrame{
          2,
          1,
          "BGR8",
          {0x10, 0x20, 0x30, 0x40, 0x50, 0x60}});
      fake_capture = backend.get();
      return backend;
    },
    [&fake_otp]() -> std::unique_ptr<OtpReader> {
      auto reader = std::make_unique<FakeOtpReader>();
      reader->result.ok = true;
      reader->result.data = valid_ox01f10_otp();
      reader->result.i2c_bus = 10;
      fake_otp = reader.get();
      return reader;
    });
  runtime.set_event_handler(
    [&events](device_manager::DeviceEvent event) {
      events.push_back(std::move(event));
    });

  auto parameters = camera_parameters();
  parameters["camera.driver.read_otp_on_start"] = true;
  parameters["camera.driver.otp_i2c_bus"] = std::int64_t{10};

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);

  ASSERT_NE(fake_capture, nullptr);
  ASSERT_NE(fake_otp, nullptr);
  EXPECT_TRUE(fake_otp->called);

  auto observer = std::make_shared<rclcpp::Node>("gmsl_runtime_otp_output_test");
  std::optional<sensor_msgs::msg::CameraInfo> camera_info;
  std::optional<std_msgs::msg::String> serial_number;
  auto info_sub = observer->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/front_gmsl/camera_info", 10,
    [&camera_info](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      camera_info = *message;
    });
  auto serial_sub = observer->create_subscription<std_msgs::msg::String>(
    "/front_gmsl/serial_number", 10,
    [&serial_number](std_msgs::msg::String::ConstSharedPtr message) {
      serial_number = *message;
    });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(observer);
  ASSERT_TRUE(
    wait_until(
      [&observer]() {return observer->count_publishers("/front_gmsl/camera_info") > 0U;}));

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(
    wait_until(
      [&executor, &camera_info, &serial_number]() {
        executor.spin_some();
        return camera_info.has_value() && serial_number.has_value();
      }));
  EXPECT_EQ(serial_number->data, "OTP-SN-001");
  ASSERT_TRUE(camera_info.has_value());
  EXPECT_EQ(camera_info->width, 2U);
  EXPECT_EQ(camera_info->height, 1U);
  EXPECT_EQ(camera_info->distortion_model, "plumb_bob");
  EXPECT_DOUBLE_EQ(camera_info->k[0], 316.4589527674 * 2.0 / 1280.0 * 0.55);
  EXPECT_DOUBLE_EQ(camera_info->k[2], 641.2050576828 * 2.0 / 1280.0);
  EXPECT_DOUBLE_EQ(camera_info->k[4], 316.4021688498 / 960.0 * 0.55);
  EXPECT_DOUBLE_EQ(camera_info->k[5], 483.1482381048 / 960.0);
  EXPECT_DOUBLE_EQ(camera_info->p[0], 316.4589527674 * 2.0 / 1280.0 * 0.55);
  EXPECT_DOUBLE_EQ(camera_info->p[2], 641.2050576828 * 2.0 / 1280.0);
  EXPECT_DOUBLE_EQ(camera_info->p[5], 316.4021688498 / 960.0 * 0.55);
  EXPECT_DOUBLE_EQ(camera_info->p[6], 483.1482381048 / 960.0);
  EXPECT_EQ(camera_info->d, (std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0}));
  ASSERT_FALSE(events.empty());
  EXPECT_EQ(events.front().values.at("serial_number"), "OTP-SN-001");
  EXPECT_EQ(events.front().values.at("otp_status"), "success");
  EXPECT_EQ(events.front().values.at("otp_i2c_bus"), "10");
  (void)info_sub;
  (void)serial_sub;
}

TEST_F(RosFixture, FallsBackToConfiguredCalibrationWhenOtpReadFails)
{
  FakeCaptureBackend * fake_capture = nullptr;
  FakeOtpReader * fake_otp = nullptr;
  std::vector<device_manager::DeviceEvent> events;
  GmslV4l2CameraRuntime runtime(
    [&fake_capture]() -> std::unique_ptr<CaptureBackend> {
      auto backend = std::make_unique<FakeCaptureBackend>();
      backend->frames.push_back(
        CapturedFrame{
          2,
          1,
          "BGR8",
          {0x10, 0x20, 0x30, 0x40, 0x50, 0x60}});
      fake_capture = backend.get();
      return backend;
    },
    [&fake_otp]() -> std::unique_ptr<OtpReader> {
      auto reader = std::make_unique<FakeOtpReader>();
      reader->result.ok = false;
      reader->result.error_message = "no otp data";
      fake_otp = reader.get();
      return reader;
    });
  runtime.set_event_handler(
    [&events](device_manager::DeviceEvent event) {
      events.push_back(std::move(event));
    });

  auto parameters = camera_parameters();
  parameters["camera.driver.read_otp_on_start"] = true;
  parameters["camera.driver.otp_i2c_bus"] = std::int64_t{10};

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);

  ASSERT_NE(fake_capture, nullptr);
  ASSERT_NE(fake_otp, nullptr);
  EXPECT_TRUE(fake_otp->called);
  ASSERT_TRUE(fake_capture->applied_config.calibration.has_value());
  EXPECT_EQ(fake_capture->applied_config.serial_number, "H190TA-I05252006");
  EXPECT_EQ(fake_capture->applied_config.calibration->width, 2U);
  EXPECT_EQ(fake_capture->applied_config.calibration->height, 1U);
  EXPECT_DOUBLE_EQ(fake_capture->applied_config.calibration->k[0], 316.4589527674);

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(wait_until([&events]() {return !events.empty();}));
  EXPECT_EQ(events.front().values.at("otp_read_on_start"), "true");
  EXPECT_EQ(events.front().values.at("otp_status"), "failed");
  EXPECT_EQ(events.front().values.at("otp_error_message"), "no otp data");
  EXPECT_EQ(events.front().values.at("otp_i2c_bus"), "10");
}

TEST_F(RosFixture, DoesNotProbeOtpWhenI2cBusIsNotConfigured)
{
  FakeCaptureBackend * fake_capture = nullptr;
  FakeOtpReader * fake_otp = nullptr;
  std::vector<device_manager::DeviceEvent> events;
  GmslV4l2CameraRuntime runtime(
    [&fake_capture]() -> std::unique_ptr<CaptureBackend> {
      auto backend = std::make_unique<FakeCaptureBackend>();
      backend->frames.push_back(
        CapturedFrame{
          2,
          1,
          "BGR8",
          {0x10, 0x20, 0x30, 0x40, 0x50, 0x60}});
      fake_capture = backend.get();
      return backend;
    },
    [&fake_otp]() -> std::unique_ptr<OtpReader> {
      auto reader = std::make_unique<FakeOtpReader>();
      fake_otp = reader.get();
      return reader;
    });
  runtime.set_event_handler(
    [&events](device_manager::DeviceEvent event) {
      events.push_back(std::move(event));
    });

  auto parameters = camera_parameters();
  parameters["camera.driver.read_otp_on_start"] = true;

  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive,
        parameters)).result,
    device_manager::TransitionResult::kSuccess);

  ASSERT_NE(fake_capture, nullptr);
  EXPECT_EQ(fake_otp, nullptr);
  ASSERT_EQ(
    runtime.request_transition(
      transition(
        device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive)).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(wait_until([&events]() {return !events.empty();}));
  EXPECT_EQ(events.front().values.at("otp_status"), "failed");
  EXPECT_EQ(
    events.front().values.at("otp_error_message"),
    "camera.driver.otp_i2c_bus must be configured to read GMSL OTP");
}

TEST(V4l2CaptureBackendTest, SelectsMultiplanarCaptureWhenDeviceOnlyExposesMplane)
{
  EXPECT_EQ(
    detail::select_capture_buffer_type(
      V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING,
      0),
    V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
  EXPECT_EQ(
    detail::select_capture_buffer_type(
      V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING,
      0),
    V4L2_BUF_TYPE_VIDEO_CAPTURE);
}

}  // namespace
}  // namespace gmsl_v4l2_camera_driver
