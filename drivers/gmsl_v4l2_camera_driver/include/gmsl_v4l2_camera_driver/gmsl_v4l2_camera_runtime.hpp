#ifndef GMSL_V4L2_CAMERA_DRIVER__GMSL_V4L2_CAMERA_RUNTIME_HPP_
#define GMSL_V4L2_CAMERA_DRIVER__GMSL_V4L2_CAMERA_RUNTIME_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <linux/videodev2.h>

#include "device_manager_core/device_manager.hpp"
#include "device_manager_msgs/msg/device_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

namespace gmsl_v4l2_camera_driver
{

struct CameraCalibration
{
  std::uint32_t width {0};
  std::uint32_t height {0};
  std::string distortion_model;
  std::vector<double> d;
  std::array<double, 9> k {};
  std::array<double, 9> r {};
  std::array<double, 12> p {};
};

struct GmslV4l2CameraConfig
{
  std::string camera_name {"gmsl_camera"};
  std::string frame_id {"gmsl_camera_link"};
  std::string serial_number;
  std::string video_device {"/dev/video0"};
  std::string image_topic {"image_raw"};
  std::string distorted_image_topic {"image_raw_distorted"};
  std::string camera_info_topic {"camera_info"};
  std::string serial_number_topic {"serial_number"};
  std::string pixel_format {"NV12"};
  std::string intrinsic_params;
  int width {1280};
  int height {720};
  int fps {10};
  int read_timeout_ms {1000};
  bool undistort_image {true};
  double undistort_focal_scale {0.85};
  int calibration_crop_y {-1};
  bool rotate_180 {false};
  bool publish_distorted_image {false};
  bool read_otp_on_start {true};
  int otp_i2c_bus {-1};
  int otp_i2c_address {0x36};
  int otp_read_offset {0x10000};
  int otp_read_size {0x2000};
  std::string otp_status {"disabled"};
  int otp_result_i2c_bus {-1};
  std::string otp_error_message;
  std::optional<CameraCalibration> calibration;
};

struct CapturedFrame
{
  int width {0};
  int height {0};
  std::string pixel_format;
  std::vector<std::uint8_t> data;
};

namespace detail
{
v4l2_buf_type select_capture_buffer_type(
  std::uint32_t capabilities,
  std::uint32_t device_caps);
}  // namespace detail

class CaptureBackend
{
public:
  virtual ~CaptureBackend() = default;
  virtual bool configure(const GmslV4l2CameraConfig & config, std::string & error_message) = 0;
  virtual std::optional<CapturedFrame> read_frame(
    std::chrono::milliseconds timeout,
    std::string & error_message) = 0;
  virtual void close() = 0;
};

struct OtpReadResult
{
  bool ok {false};
  std::vector<std::uint8_t> data;
  int i2c_bus {-1};
  std::string error_message;
};

class OtpReader
{
public:
  virtual ~OtpReader() = default;
  virtual OtpReadResult read(const GmslV4l2CameraConfig & config) = 0;
};

class I2cOtpReader final : public OtpReader
{
public:
  OtpReadResult read(const GmslV4l2CameraConfig & config) override;
};

struct UndistortMap
{
  int width {0};
  int height {0};
  std::string distortion_model;
  std::vector<double> d;
  std::array<double, 9> source_k {};
  std::array<double, 9> output_k {};
  std::vector<float> source_x;
  std::vector<float> source_y;
};

class V4l2CaptureBackend final : public CaptureBackend
{
public:
  ~V4l2CaptureBackend() override;

  bool configure(const GmslV4l2CameraConfig & config, std::string & error_message) override;
  std::optional<CapturedFrame> read_frame(
    std::chrono::milliseconds timeout,
    std::string & error_message) override;
  void close() override;

private:
  struct Buffer
  {
    struct Plane
    {
      void * start {nullptr};
      std::size_t length {0};
    };

    std::vector<Plane> planes;
  };

  int fd_ {-1};
  v4l2_buf_type buffer_type_ {V4L2_BUF_TYPE_VIDEO_CAPTURE};
  std::string pixel_format_;
  int width_ {0};
  int height_ {0};
  std::vector<Buffer> buffers_;
};

class GmslV4l2CameraRuntime final : public device_manager::DeviceRuntimeBase
{
public:
  using CaptureBackendFactory = std::function<std::unique_ptr<CaptureBackend>()>;
  using OtpReaderFactory = std::function<std::unique_ptr<OtpReader>()>;

  explicit GmslV4l2CameraRuntime(
    CaptureBackendFactory capture_factory = {},
    OtpReaderFactory otp_reader_factory = {});
  ~GmslV4l2CameraRuntime() override;

  void initialize(const device_manager::DeviceDefinition & definition) override;
  device_manager::LifecycleState state() const override;
  std::vector<device_manager::TransitionRequest> parameter_transitions(
    device_manager::LifecycleState state,
    const device_manager::ParameterMap & parameters,
    const device_manager::ParameterMap &) override
  {
    return device_manager::standard_parameter_transitions(state, parameters);
  }
  device_manager::TransitionOutcome materialize(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome dematerialize() override;
  void set_event_handler(device_manager::EventHandler handler) override;
  void register_hook(device_manager::HookRegistrar registrar) override;

private:
  device_manager::TransitionOutcome configure(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome activate() override;
  device_manager::TransitionOutcome deactivate() override;
  device_manager::TransitionOutcome cleanup(
    const device_manager::ParameterMap & parameters) override;

  GmslV4l2CameraConfig config_from(
    const device_manager::ParameterMap & parameters,
    std::string & error_message) const;
  void capture_loop();
  void publish_frame(const CapturedFrame & frame);
  std::shared_ptr<const UndistortMap> undistort_map_for(
    const CameraCalibration & source_calibration,
    const CameraCalibration & output_calibration,
    int width,
    int height);
  void emit_event(device_manager::DeviceEvent event);
  void emit_connection_event(
    bool connected,
    const std::string & code,
    const std::string & message,
    device_manager::LifecycleState target_state);
  void stop_capture_thread();
  void reset_ros_output();

  mutable std::mutex mutex_;
  CaptureBackendFactory capture_factory_;
  OtpReaderFactory otp_reader_factory_;
  GmslV4l2CameraConfig config_;
  std::string device_id_;
  device_manager::LifecycleState state_;
  device_manager::EventHandler event_handler_;
  std::unique_ptr<CaptureBackend> capture_;
  std::atomic<bool> running_ {false};
  std::thread capture_thread_;
  std::optional<bool> last_connected_;
  std::shared_ptr<const UndistortMap> undistort_map_;
  bool logged_first_frame_ {false};
  bool logged_first_publish_ {false};

  std::shared_ptr<rclcpp::Node> output_node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr distorted_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_number_publisher_;
  rclcpp::Publisher<device_manager_msgs::msg::DeviceEvent>::SharedPtr event_publisher_;
};

}  // namespace gmsl_v4l2_camera_driver

#endif  // GMSL_V4L2_CAMERA_DRIVER__GMSL_V4L2_CAMERA_RUNTIME_HPP_
