#include "gmsl_v4l2_camera_driver/gmsl_v4l2_camera_runtime.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <fcntl.h>
#include <memory>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utility>
#include <variant>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

namespace gmsl_v4l2_camera_driver
{
namespace
{

using device_manager::LifecycleState;
using device_manager::ParameterMap;
using device_manager::TransitionOutcome;
using device_manager::TransitionRequest;
using device_manager::TransitionResult;

template<typename T>
std::optional<T> parameter_as(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    return std::nullopt;
  }
  if (const auto value = std::get_if<T>(&found->second)) {
    return *value;
  }
  error_message = "invalid parameter type: " + name;
  return std::nullopt;
}

std::optional<std::string> string_parameter(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  return parameter_as<std::string>(parameters, name, error_message);
}

std::optional<std::int64_t> integer_parameter(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  return parameter_as<std::int64_t>(parameters, name, error_message);
}

std::optional<bool> bool_parameter(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  return parameter_as<bool>(parameters, name, error_message);
}

std::optional<double> double_parameter(
  const ParameterMap & parameters,
  const std::string & name,
  std::string & error_message)
{
  return parameter_as<double>(parameters, name, error_message);
}

std::optional<int> parse_int_value(const std::string & value)
{
  if (value.empty()) {
    return std::nullopt;
  }
  char * end = nullptr;
  const auto parsed = std::strtol(value.c_str(), &end, 0);
  if (end == value.c_str()) {
    return std::nullopt;
  }
  return static_cast<int>(parsed);
}

std::string first_string(
  const ParameterMap & parameters,
  const std::vector<std::string> & names,
  const std::string & fallback,
  std::string & error_message)
{
  for (const auto & name : names) {
    const auto value = string_parameter(parameters, name, error_message);
    if (!error_message.empty()) {
      return fallback;
    }
    if (value && !value->empty()) {
      return *value;
    }
  }
  return fallback;
}

bool first_bool(
  const ParameterMap & parameters,
  const std::vector<std::string> & names,
  bool fallback,
  std::string & error_message)
{
  for (const auto & name : names) {
    const auto value = bool_parameter(parameters, name, error_message);
    if (!error_message.empty()) {
      return fallback;
    }
    if (value) {
      return *value;
    }
  }
  return fallback;
}

int first_int(
  const ParameterMap & parameters,
  const std::vector<std::string> & names,
  int fallback,
  std::string & error_message)
{
  for (const auto & name : names) {
    const auto value = integer_parameter(parameters, name, error_message);
    if (!error_message.empty()) {
      return fallback;
    }
    if (value && *value > 0) {
      return static_cast<int>(*value);
    }
  }
  return fallback;
}

int first_number(
  const ParameterMap & parameters,
  const std::vector<std::string> & names,
  int fallback,
  std::string & error_message)
{
  for (const auto & name : names) {
    if (const auto value = integer_parameter(parameters, name, error_message)) {
      return static_cast<int>(*value);
    }
    if (!error_message.empty()) {
      if (const auto string_value = string_parameter(parameters, name, error_message = "")) {
        if (const auto parsed = parse_int_value(*string_value)) {
          return *parsed;
        }
        error_message = "invalid integer parameter: " + name;
      }
      return fallback;
    }
  }
  return fallback;
}

double first_double(
  const ParameterMap & parameters,
  const std::vector<std::string> & names,
  double fallback,
  std::string & error_message)
{
  for (const auto & name : names) {
    if (const auto value = double_parameter(parameters, name, error_message)) {
      return *value;
    }
    if (!error_message.empty()) {
      if (const auto integer_value = integer_parameter(parameters, name, error_message = "")) {
        return static_cast<double>(*integer_value);
      }
      if (const auto string_value = string_parameter(parameters, name, error_message = "")) {
        char * end = nullptr;
        const auto parsed = std::strtod(string_value->c_str(), &end);
        if (end != string_value->c_str()) {
          return parsed;
        }
        error_message = "invalid double parameter: " + name;
      }
      return fallback;
    }
  }
  return fallback;
}

std::string trim_copy(const std::string & value)
{
  const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
        return std::isspace(ch);
    });
  const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
        return std::isspace(ch);
    }).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

std::string lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
      return static_cast<char>(std::tolower(ch));
    });
  return value;
}

bool names_fisheye_model(const std::optional<std::string> & value)
{
  if (!value) {
    return false;
  }
  const auto normalized = lower_copy(trim_copy(*value));
  return normalized == "fisheye" || normalized == "equidistant" ||
         normalized == "kannala-brandt" || normalized == "kannala_brandt" ||
         normalized == "kb";
}

std::optional<std::string> string_field(const std::string & text, const std::string & key)
{
  const std::regex quoted_pattern(
    "\"" + key + "\"\\s*:\\s*\"([^\"]*)\"",
    std::regex_constants::icase);
  std::smatch match;
  if (std::regex_search(text, match, quoted_pattern)) {
    return match[1].str();
  }

  const std::regex bare_pattern(
    "(^|[\\{,;\\s])\"?" + key + "\"?\\s*[:=]\\s*([^,;\\}\\n\\r]+)",
    std::regex_constants::icase);
  if (std::regex_search(text, match, bare_pattern)) {
    return trim_copy(match[2].str());
  }
  return std::nullopt;
}

std::optional<int> integer_field(const std::string & text, const std::string & key)
{
  const auto value = string_field(text, key);
  if (!value || value->empty()) {
    return std::nullopt;
  }
  char * end = nullptr;
  const auto parsed = std::strtol(value->c_str(), &end, 10);
  if (end == value->c_str()) {
    return std::nullopt;
  }
  return static_cast<int>(parsed);
}

std::optional<double> double_field(const std::string & text, const std::string & key)
{
  const auto value = string_field(text, key);
  if (!value || value->empty()) {
    return std::nullopt;
  }
  char * end = nullptr;
  const auto parsed = std::strtod(value->c_str(), &end);
  if (end == value->c_str()) {
    return std::nullopt;
  }
  return parsed;
}

std::optional<std::vector<double>> double_array_field(
  const std::string & text,
  const std::string & key)
{
  const std::regex pattern(
    "(^|[\\{,;\\s])\"?" + key + "\"?\\s*[:=]\\s*\\[([^\\]]*)\\]",
    std::regex_constants::icase);
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) {
    return std::nullopt;
  }

  std::vector<double> values;
  const std::string array_text = match[2].str();
  const std::regex number_pattern(R"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)");
  for (auto it = std::sregex_iterator(array_text.begin(), array_text.end(), number_pattern);
    it != std::sregex_iterator(); ++it)
  {
    values.push_back(std::strtod((*it)[0].str().c_str(), nullptr));
  }
  return values;
}

template<std::size_t Size>
bool copy_exact_array(
  const std::optional<std::vector<double>> & values,
  std::array<double, Size> & output)
{
  if (!values || values->size() != Size) {
    return false;
  }
  std::copy(values->begin(), values->end(), output.begin());
  return true;
}

CameraCalibration default_calibration()
{
  CameraCalibration calibration;
  calibration.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  calibration.p = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  return calibration;
}

std::optional<CameraCalibration> parse_camera_calibration(
  const std::string & intrinsic_params,
  std::string & error_message)
{
  if (trim_copy(intrinsic_params).empty()) {
    return std::nullopt;
  }

  auto calibration = default_calibration();
  if (const auto width = integer_field(intrinsic_params, "width")) {
    calibration.width = static_cast<std::uint32_t>(*width);
  }
  if (const auto height = integer_field(intrinsic_params, "height")) {
    calibration.height = static_cast<std::uint32_t>(*height);
  }

  if (const auto distortion_model = string_field(intrinsic_params, "distortion_model");
    names_fisheye_model(distortion_model))
  {
    calibration.distortion_model = "equidistant";
  } else if (distortion_model) {
    calibration.distortion_model = *distortion_model;
  } else if (
    names_fisheye_model(string_field(intrinsic_params, "model")) ||
    names_fisheye_model(string_field(intrinsic_params, "type")))
  {
    calibration.distortion_model = "equidistant";
  } else {
    calibration.distortion_model = "plumb_bob";
  }

  const auto d_values = double_array_field(intrinsic_params, "d");
  if (d_values) {
    calibration.d = *d_values;
  }

  const auto k_values = double_array_field(intrinsic_params, "k");
  if (!copy_exact_array(k_values, calibration.k)) {
    const auto fx = double_field(intrinsic_params, "fx");
    const auto fy = double_field(intrinsic_params, "fy");
    const auto cx = double_field(intrinsic_params, "cx");
    const auto cy = double_field(intrinsic_params, "cy");
    if (!fx || !fy || !cx || !cy) {
      error_message = "invalid camera.driver.intrinsic_params: missing k[9] or fx/fy/cx/cy";
      return std::nullopt;
    }
    calibration.k = {*fx, 0.0, *cx, 0.0, *fy, *cy, 0.0, 0.0, 1.0};
  }

  if (!copy_exact_array(double_array_field(intrinsic_params, "r"), calibration.r)) {
    calibration.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  }
  if (!copy_exact_array(double_array_field(intrinsic_params, "p"), calibration.p)) {
    calibration.p = {
      calibration.k[0], 0.0, calibration.k[2], 0.0,
      0.0, calibration.k[4], calibration.k[5], 0.0,
      0.0, 0.0, 1.0, 0.0};
  }

  if (calibration.d.empty()) {
    if (calibration.distortion_model == "equidistant") {
      for (const auto key : {"k1", "k2", "k3", "k4"}) {
        const auto value = double_field(intrinsic_params, key);
        if (value) {
          calibration.d.push_back(*value);
        }
      }
    } else {
      for (const auto key : {"k1", "k2", "p1", "p2", "k3"}) {
        const auto value = double_field(intrinsic_params, key);
        if (value) {
          calibration.d.push_back(*value);
        }
      }
    }
  }

  return calibration;
}

std::uint16_t read_le_u16(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::uint16_t>(data[offset]) |
         static_cast<std::uint16_t>(data[offset + 1U] << 8U);
}

std::uint32_t read_le_u32(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  return static_cast<std::uint32_t>(data[offset]) |
         (static_cast<std::uint32_t>(data[offset + 1U]) << 8U) |
         (static_cast<std::uint32_t>(data[offset + 2U]) << 16U) |
         (static_cast<std::uint32_t>(data[offset + 3U]) << 24U);
}

double read_le_double(const std::vector<std::uint8_t> & data, std::size_t offset)
{
  std::uint64_t raw = 0U;
  for (std::size_t index = 0; index < 8U; ++index) {
    raw |= static_cast<std::uint64_t>(data[offset + index]) << (index * 8U);
  }
  double value = 0.0;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

std::uint32_t crc32(const std::vector<std::uint8_t> & data, std::size_t begin, std::size_t end)
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

bool all_bytes_are(const std::vector<std::uint8_t> & data, std::uint8_t value)
{
  return std::all_of(data.begin(), data.end(), [value](std::uint8_t byte) {
             return byte == value;
    });
}

std::string read_otp_serial_number(const std::vector<std::uint8_t> & data)
{
  std::string serial_number;
  for (std::size_t offset = 0x120U; offset < 0x130U; ++offset) {
    const auto value = data[offset];
    if (value == 0x00U || value == 0xffU) {
      break;
    }
    if (value < 0x20U || value > 0x7eU) {
      continue;
    }
    serial_number.push_back(static_cast<char>(value));
  }
  return serial_number;
}

std::optional<CameraCalibration> decode_ox01f10_otp(
  const std::vector<std::uint8_t> & data,
  std::string & serial_number,
  std::string & error_message)
{
  if (data.size() < 0x145U) {
    error_message = "otp data is shorter than OX01F10 calibration block";
    return std::nullopt;
  }
  if (all_bytes_are(data, 0x00U) || all_bytes_are(data, 0xffU)) {
    error_message = "otp data is blank";
    return std::nullopt;
  }

  const auto crc4 = crc32(data, 0x60U, 0x11cU);
  const auto stored_crc4 = read_le_u32(data, 0x11cU);
  if (crc4 != stored_crc4) {
    std::ostringstream stream;
    stream << "otp crc32_4 mismatch: calculated 0x" << std::hex << crc4 <<
      " stored 0x" << stored_crc4;
    error_message = stream.str();
    return std::nullopt;
  }

  const auto crc5 = crc32(data, 0x120U, 0x130U);
  const auto stored_crc5 = read_le_u32(data, 0x141U);
  if (crc5 != stored_crc5) {
    std::ostringstream stream;
    stream << "otp crc32_5 mismatch: calculated 0x" << std::hex << crc5 <<
      " stored 0x" << stored_crc5;
    error_message = stream.str();
    return std::nullopt;
  }

  serial_number = read_otp_serial_number(data);
  if (serial_number.empty()) {
    error_message = "otp serial number is empty";
    return std::nullopt;
  }

  auto calibration = default_calibration();
  calibration.width = read_le_u16(data, 0x60U);
  calibration.height = read_le_u16(data, 0x62U);
  const auto model = data[0x64U];
  calibration.k = {
    read_le_double(data, 0x65U), 0.0, read_le_double(data, 0x75U),
    0.0, read_le_double(data, 0x6dU), read_le_double(data, 0x7dU),
    0.0, 0.0, 1.0};
  calibration.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  calibration.p = {
    calibration.k[0], 0.0, calibration.k[2], 0.0,
    0.0, calibration.k[4], calibration.k[5], 0.0,
    0.0, 0.0, 1.0, 0.0};

  if (model == 0x02U) {
    calibration.distortion_model = "equidistant";
    calibration.d = {
      read_le_double(data, 0xc5U),
      read_le_double(data, 0xcdU),
      read_le_double(data, 0xd5U),
      read_le_double(data, 0xddU)};
  } else {
    calibration.distortion_model = "plumb_bob";
    calibration.d = {
      read_le_double(data, 0x85U),
      read_le_double(data, 0x8dU),
      read_le_double(data, 0x95U),
      read_le_double(data, 0x9dU),
      read_le_double(data, 0xa5U)};
  }
  return calibration;
}

bool event_needs_recovery(const device_manager::DeviceEvent & event)
{
  return event.level == device_manager::EventLevel::kError ||
         event.level == device_manager::EventLevel::kStale;
}

std::vector<TransitionRequest> recovery_requests(LifecycleState from, LifecycleState target)
{
  if (target == LifecycleState::kFinalized) {
    if (from == LifecycleState::kActive) {
      return {
        {LifecycleState::kActive, LifecycleState::kInactive, {}},
        {LifecycleState::kInactive, LifecycleState::kUnconfigured, {}},
        {LifecycleState::kUnconfigured, LifecycleState::kFinalized, {}},
      };
    }
    if (from == LifecycleState::kInactive) {
      return {
        {LifecycleState::kInactive, LifecycleState::kUnconfigured, {}},
        {LifecycleState::kUnconfigured, LifecycleState::kFinalized, {}},
      };
    }
    if (from == LifecycleState::kUnconfigured) {
      return {{LifecycleState::kUnconfigured, LifecycleState::kFinalized, {}}};
    }
  }
  if (from == LifecycleState::kActive && target == LifecycleState::kUnconfigured) {
    return {
      {LifecycleState::kActive, LifecycleState::kInactive, {}},
      {LifecycleState::kInactive, LifecycleState::kUnconfigured, {}},
    };
  }
  if (from == LifecycleState::kInactive && target == LifecycleState::kUnconfigured) {
    return {{LifecycleState::kInactive, LifecycleState::kUnconfigured, {}}};
  }
  return {};
}

std::vector<std::uint8_t> convert_to_bgr8(const CapturedFrame & frame)
{
  if (frame.pixel_format == "BGR8" || frame.pixel_format == "bgr8") {
    return frame.data;
  }
  if (frame.pixel_format == "RGB8" || frame.pixel_format == "rgb8") {
    const cv::Mat rgb(frame.height, frame.width, CV_8UC3, const_cast<std::uint8_t *>(
        frame.data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    std::vector<std::uint8_t> output(bgr.total() * bgr.elemSize());
    std::memcpy(output.data(), bgr.data, output.size());
    return output;
  }
  if (frame.pixel_format == "MONO8" || frame.pixel_format == "GRAY8" ||
    frame.pixel_format == "GREY")
  {
    const cv::Mat mono(frame.height, frame.width, CV_8UC1, const_cast<std::uint8_t *>(
        frame.data.data()));
    cv::Mat bgr;
    cv::cvtColor(mono, bgr, cv::COLOR_GRAY2BGR);
    std::vector<std::uint8_t> output(bgr.total() * bgr.elemSize());
    std::memcpy(output.data(), bgr.data, output.size());
    return output;
  }
  if (frame.pixel_format == "NV12" || frame.pixel_format == "NV21") {
    const auto width = static_cast<std::size_t>(frame.width);
    const auto height = static_cast<std::size_t>(frame.height);
    const auto y_size = width * height;
    if (frame.data.size() < y_size + y_size / 2U) {
      return {};
    }
    const cv::Mat yuv(static_cast<int>(height + height / 2U), static_cast<int>(width), CV_8UC1,
      const_cast<std::uint8_t *>(frame.data.data()));
    cv::Mat bgr;
    cv::cvtColor(yuv, bgr, frame.pixel_format == "NV12" ? cv::COLOR_YUV2BGR_NV12 :
      cv::COLOR_YUV2BGR_NV21);
    std::vector<std::uint8_t> output(bgr.total() * bgr.elemSize());
    std::memcpy(output.data(), bgr.data, output.size());
    return output;
  }
  if (frame.pixel_format == "YUYV" || frame.pixel_format == "YUY2") {
    const cv::Mat yuyv(frame.height, frame.width, CV_8UC2, const_cast<std::uint8_t *>(
        frame.data.data()));
    cv::Mat bgr;
    cv::cvtColor(yuyv, bgr, cv::COLOR_YUV2BGR_YUY2);
    std::vector<std::uint8_t> output(bgr.total() * bgr.elemSize());
    std::memcpy(output.data(), bgr.data, output.size());
    return output;
  }
  if (frame.pixel_format == "UYVY") {
    const cv::Mat uyvy(frame.height, frame.width, CV_8UC2, const_cast<std::uint8_t *>(
        frame.data.data()));
    cv::Mat bgr;
    cv::cvtColor(uyvy, bgr, cv::COLOR_YUV2BGR_UYVY);
    std::vector<std::uint8_t> output(bgr.total() * bgr.elemSize());
    std::memcpy(output.data(), bgr.data, output.size());
    return output;
  }
  return {};
}

bool calibration_matches_map(
  const UndistortMap & map,
  const CameraCalibration & source_calibration,
  const CameraCalibration & output_calibration,
  int width,
  int height)
{
  return map.width == width && map.height == height &&
         map.distortion_model == source_calibration.distortion_model &&
         map.d == source_calibration.d && map.source_k == source_calibration.k &&
         map.output_k == output_calibration.k;
}

bool supports_undistortion(const CameraCalibration & calibration)
{
  const auto fx = calibration.k[0];
  const auto fy = calibration.k[4];
  return fx > 0.0 && fy > 0.0 &&
         (calibration.distortion_model == "equidistant" ||
         calibration.distortion_model == "fisheye" ||
         calibration.distortion_model == "plumb_bob");
}

CameraCalibration scale_calibration_to_frame(
  const CameraCalibration & calibration,
  int width,
  int height,
  int configured_crop_y)
{
  auto scaled = calibration;
  if (width <= 0 || height <= 0 || calibration.width == 0U || calibration.height == 0U) {
    scaled.width = static_cast<std::uint32_t>(std::max(width, 0));
    scaled.height = static_cast<std::uint32_t>(std::max(height, 0));
    return scaled;
  }

  if (static_cast<std::uint32_t>(width) == calibration.width &&
    static_cast<std::uint32_t>(height) < calibration.height)
  {
    const auto max_crop_y = static_cast<int>(calibration.height) - height;
    const auto crop_y = configured_crop_y >= 0 ?
      static_cast<double>(std::clamp(configured_crop_y, 0, max_crop_y)) :
      static_cast<double>(max_crop_y) / 2.0;
    scaled.height = static_cast<std::uint32_t>(height);
    scaled.k[5] -= crop_y;
    scaled.p[6] -= crop_y;
    return scaled;
  }

  if (static_cast<std::uint32_t>(height) == calibration.height &&
    static_cast<std::uint32_t>(width) < calibration.width)
  {
    const auto crop_x =
      (static_cast<double>(calibration.width) - static_cast<double>(width)) / 2.0;
    scaled.width = static_cast<std::uint32_t>(width);
    scaled.k[2] -= crop_x;
    scaled.p[2] -= crop_x;
    return scaled;
  }

  const auto x_scale = static_cast<double>(width) / static_cast<double>(calibration.width);
  const auto y_scale = static_cast<double>(height) / static_cast<double>(calibration.height);

  scaled.width = static_cast<std::uint32_t>(width);
  scaled.height = static_cast<std::uint32_t>(height);
  scaled.k[0] *= x_scale;
  scaled.k[2] *= x_scale;
  scaled.k[4] *= y_scale;
  scaled.k[5] *= y_scale;
  scaled.p[0] *= x_scale;
  scaled.p[2] *= x_scale;
  scaled.p[3] *= x_scale;
  scaled.p[5] *= y_scale;
  scaled.p[6] *= y_scale;
  scaled.p[7] *= y_scale;
  return scaled;
}

CameraCalibration wide_fov_output_calibration(
  const CameraCalibration & calibration,
  double focal_scale)
{
  auto output = calibration;
  const auto scale = std::clamp(focal_scale, 0.1, 1.0);
  output.k[0] *= scale;
  output.k[4] *= scale;
  output.p[0] *= scale;
  output.p[5] *= scale;
  return output;
}

void append_trace_line(const std::string & line)
{
  std::ofstream stream("/tmp/gmsl_v4l2_trace.log", std::ios::app);
  stream << line << '\n';
}

std::shared_ptr<UndistortMap> build_undistort_map(
  const CameraCalibration & source_calibration,
  const CameraCalibration & output_calibration,
  int width,
  int height)
{
  if (!supports_undistortion(source_calibration) || width <= 0 || height <= 0) {
    return {};
  }

  auto map = std::make_shared<UndistortMap>();
  map->width = width;
  map->height = height;
  map->distortion_model = source_calibration.distortion_model;
  map->d = source_calibration.d;
  map->source_k = source_calibration.k;
  map->output_k = output_calibration.k;
  const auto pixel_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  map->source_x.resize(pixel_count, -1.0F);
  map->source_y.resize(pixel_count, -1.0F);

  const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    source_calibration.k[0], source_calibration.k[1], source_calibration.k[2],
    source_calibration.k[3], source_calibration.k[4], source_calibration.k[5],
    source_calibration.k[6], source_calibration.k[7], source_calibration.k[8]);
  const cv::Mat output_camera_matrix = (cv::Mat_<double>(3, 3) <<
    output_calibration.k[0], output_calibration.k[1], output_calibration.k[2],
    output_calibration.k[3], output_calibration.k[4], output_calibration.k[5],
    output_calibration.k[6], output_calibration.k[7], output_calibration.k[8]);
  const cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat map_x;
  cv::Mat map_y;

  if (source_calibration.distortion_model == "equidistant" ||
    source_calibration.distortion_model == "fisheye")
  {
    const cv::Mat distortion = (cv::Mat_<double>(4, 1) <<
      (source_calibration.d.size() > 0U ? source_calibration.d[0] : 0.0),
      (source_calibration.d.size() > 1U ? source_calibration.d[1] : 0.0),
      (source_calibration.d.size() > 2U ? source_calibration.d[2] : 0.0),
      (source_calibration.d.size() > 3U ? source_calibration.d[3] : 0.0));
    cv::fisheye::initUndistortRectifyMap(
      camera_matrix,
      distortion,
      identity,
      output_camera_matrix,
      cv::Size(width, height),
      CV_32FC1,
      map_x,
      map_y);
  } else {
    const cv::Mat distortion = (cv::Mat_<double>(1, 5) <<
      (source_calibration.d.size() > 0U ? source_calibration.d[0] : 0.0),
      (source_calibration.d.size() > 1U ? source_calibration.d[1] : 0.0),
      (source_calibration.d.size() > 2U ? source_calibration.d[2] : 0.0),
      (source_calibration.d.size() > 3U ? source_calibration.d[3] : 0.0),
      (source_calibration.d.size() > 4U ? source_calibration.d[4] : 0.0));
    cv::initUndistortRectifyMap(
      camera_matrix,
      distortion,
      identity,
      output_camera_matrix,
      cv::Size(width, height),
      CV_32FC1,
      map_x,
      map_y);
  }

  for (int row = 0; row < height; ++row) {
    const auto * map_x_row = map_x.ptr<float>(row);
    const auto * map_y_row = map_y.ptr<float>(row);
    const auto row_offset = static_cast<std::size_t>(row) * static_cast<std::size_t>(width);
    for (int column = 0; column < width; ++column) {
      const auto index = row_offset + static_cast<std::size_t>(column);
      map->source_x[index] = map_x_row[column];
      map->source_y[index] = map_y_row[column];
    }
  }
  return map;
}

std::vector<std::uint8_t> remap_bgr8(
  const std::vector<std::uint8_t> & input,
  int width,
  int height,
  const UndistortMap & map)
{
  std::vector<std::uint8_t> output(input.size(), 0U);
  const auto pixel_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  if (input.size() != pixel_count * 3U || map.source_x.size() != pixel_count ||
    map.source_y.size() != pixel_count)
  {
    return output;
  }

  const cv::Mat input_image(height, width, CV_8UC3, const_cast<std::uint8_t *>(input.data()));
  const cv::Mat map_x(height, width, CV_32FC1, const_cast<float *>(map.source_x.data()));
  const cv::Mat map_y(height, width, CV_32FC1, const_cast<float *>(map.source_y.data()));
  cv::Mat remapped;
  cv::remap(input_image, remapped, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
  std::memcpy(output.data(), remapped.data, output.size());
  return output;
}

void set_zero_distortion(sensor_msgs::msg::CameraInfo & camera_info)
{
  camera_info.distortion_model = "plumb_bob";
  camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
}

device_manager_msgs::msg::DeviceEvent to_ros_event(
  const device_manager::DeviceEvent & event,
  const rclcpp::Time & stamp)
{
  device_manager_msgs::msg::DeviceEvent message;
  message.timestamp = stamp;
  message.level = static_cast<std::uint8_t>(event.level);
  message.code = event.code;
  message.message = event.message;
  message.source = event.source;
  message.target_state = static_cast<std::uint8_t>(event.target_state);
  for (const auto & [key, value] : event.values) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    message.values.push_back(std::move(item));
  }
  return message;
}

bool transfer_i2c_messages(int fd, i2c_msg * messages, std::uint32_t message_count)
{
  i2c_rdwr_ioctl_data transfer;
  transfer.msgs = messages;
  transfer.nmsgs = message_count;
  return ioctl(fd, I2C_RDWR, &transfer) >= 0;
}

bool i2c_write_register8(
  int fd,
  std::uint16_t address,
  std::uint16_t register_address,
  std::uint8_t value,
  std::string & error_message)
{
  std::uint8_t bytes[3] = {
    static_cast<std::uint8_t>((register_address >> 8U) & 0xffU),
    static_cast<std::uint8_t>(register_address & 0xffU),
    value};
  i2c_msg message;
  message.addr = address;
  message.flags = 0;
  message.len = sizeof(bytes);
  message.buf = bytes;
  if (transfer_i2c_messages(fd, &message, 1U)) {
    return true;
  }
  error_message = std::strerror(errno);
  return false;
}

bool i2c_read_register_block16(
  int fd,
  std::uint16_t address,
  std::uint16_t register_address,
  std::vector<std::uint8_t> & output,
  std::string & error_message)
{
  std::uint8_t register_bytes[2] = {
    static_cast<std::uint8_t>((register_address >> 8U) & 0xffU),
    static_cast<std::uint8_t>(register_address & 0xffU)};
  i2c_msg messages[2];
  messages[0].addr = address;
  messages[0].flags = 0;
  messages[0].len = sizeof(register_bytes);
  messages[0].buf = register_bytes;
  messages[1].addr = address;
  messages[1].flags = I2C_M_RD;
  messages[1].len = static_cast<decltype(messages[1].len)>(output.size());
  messages[1].buf = output.data();
  if (transfer_i2c_messages(fd, messages, 2U)) {
    return true;
  }
  error_message = std::strerror(errno);
  return false;
}

bool ox01f10_write_registers(
  int fd,
  std::uint16_t address,
  const std::vector<std::pair<std::uint16_t, std::uint8_t>> & registers,
  const std::string & stage,
  std::string & error_message)
{
  for (const auto & [register_address, value] : registers) {
    std::string transfer_error;
    if (i2c_write_register8(fd, address, register_address, value, transfer_error)) {
      continue;
    }
    std::ostringstream stream;
    stream << stage << ": write 0x" << std::hex << register_address << " failed: " <<
      transfer_error;
    error_message = stream.str();
    return false;
  }
  return true;
}

bool ox01f10_trigger_command(
  int fd,
  std::uint16_t address,
  const std::string & stage,
  std::string & error_message)
{
  if (!i2c_write_register8(fd, address, 0x8160U, 0x01U, error_message)) {
    error_message = stage + ": trigger failed: " + error_message;
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  return true;
}

bool initialize_ox01f10_otp_access(
  int fd,
  std::uint16_t address,
  std::string & error_message)
{
  const std::vector<std::pair<std::uint16_t, std::uint8_t>> setup_registers = {
    {0xa10aU, 0x00U},
    {0xa11eU, 0x7fU},
    {0xa11dU, 0x00U},
    {0xa110U, 0x80U},
    {0xa10fU, 0x18U},
    {0xa10eU, 0x4cU},
    {0xa10dU, 0x00U},
    {0x8181U, 0x00U},
    {0xe400U, 0x81U},
    {0xe401U, 0x00U},
    {0xe402U, 0x00U},
    {0xe403U, 0x01U},
    {0xe404U, 0x15U},
    {0xe410U, 0x33U},
    {0xe411U, 0x44U}};
  return ox01f10_write_registers(
    fd,
    address,
    setup_registers,
    "initialize OX01F10 OTP access",
    error_message) &&
         ox01f10_trigger_command(fd, address, "initialize OX01F10 OTP access", error_message);
}

bool read_ox01f10_otp_window(
  int fd,
  std::uint16_t address,
  int flash_address,
  std::vector<std::uint8_t> & output,
  std::string & error_message)
{
  const auto size = static_cast<int>(output.size());
  if (size <= 0 || size > 256) {
    error_message = "OX01F10 OTP window size must be between 1 and 256 bytes";
    return false;
  }

  const std::vector<std::pair<std::uint16_t, std::uint8_t>> command_registers = {
    {0x8181U, 0x00U},
    {0xe400U, 0x81U},
    {0xe401U, 0x00U},
    {0xe402U, 0x00U},
    {0xe403U, 0x05U},
    {0xe404U, 0x12U},
    {0xe405U, static_cast<std::uint8_t>((flash_address >> 16U) & 0xffU)},
    {0xe406U, static_cast<std::uint8_t>((flash_address >> 8U) & 0xffU)},
    {0xe407U, static_cast<std::uint8_t>(flash_address & 0xffU)},
    {0xe408U, static_cast<std::uint8_t>((size >> 8U) & 0xffU)},
    {0xe409U, static_cast<std::uint8_t>(size & 0xffU)}};
  if (!ox01f10_write_registers(
      fd,
      address,
      command_registers,
      "read OX01F10 OTP window",
      error_message) ||
    !ox01f10_trigger_command(fd, address, "read OX01F10 OTP window", error_message))
  {
    return false;
  }

  std::string transfer_error;
  if (i2c_read_register_block16(fd, address, 0xe700U, output, transfer_error)) {
    return true;
  }
  std::ostringstream stream;
  stream << "read OX01F10 OTP window 0x" << std::hex << flash_address << " failed: " <<
    transfer_error;
  error_message = stream.str();
  return false;
}

bool soft_reset_ox01f10_sensor(int fd, std::uint16_t address, std::string & error_message)
{
  const std::vector<std::pair<std::uint16_t, std::uint8_t>> reset_registers = {
    {0x0107U, 0x01U},
    {0x8018U, 0x00U},
    {0x8019U, 0x00U},
    {0x801aU, 0x00U},
    {0x801bU, 0xc0U},
    {0x0100U, 0x00U},
    {0x0100U, 0x01U}};
  for (const auto & [register_address, value] : reset_registers) {
    std::string transfer_error;
    if (!i2c_write_register8(fd, address, register_address, value, transfer_error)) {
      std::ostringstream stream;
      stream << "soft reset OX01F10: write 0x" << std::hex << register_address <<
        " failed: " << transfer_error;
      error_message = stream.str();
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(register_address == 0x0107U ? 150 : 80));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(220));
  return true;
}

OtpReadResult read_otp_from_bus(const GmslV4l2CameraConfig & config, int bus)
{
  OtpReadResult result;
  result.i2c_bus = bus;
  const std::string path = "/dev/i2c-" + std::to_string(bus);
  const int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
  if (fd < 0) {
    result.error_message = path + ": " + std::strerror(errno);
    return result;
  }

  std::string error_message;
  const auto address = static_cast<std::uint16_t>(config.otp_i2c_address);
  bool reset_required = false;
  const auto fail = [&](const std::string & message) {
      std::string reset_error;
      if (reset_required && !soft_reset_ox01f10_sensor(fd, address, reset_error)) {
        result.error_message = message + "; " + reset_error;
      } else {
        result.error_message = message;
      }
      result.data.clear();
      close(fd);
      return result;
    };

  if (!initialize_ox01f10_otp_access(fd, address, error_message)) {
    close(fd);
    result.error_message = path + ": " + error_message;
    return result;
  }
  reset_required = true;

  const auto read_size = std::max(0, config.otp_read_size);
  result.data.resize(static_cast<std::size_t>(read_size));
  constexpr int kChunkSize = 256;
  for (int offset = 0; offset < read_size; offset += kChunkSize) {
    const int chunk_size = std::min(kChunkSize, read_size - offset);
    std::vector<std::uint8_t> chunk(static_cast<std::size_t>(chunk_size));
    if (!read_ox01f10_otp_window(
        fd, address, config.otp_read_offset + offset, chunk, error_message))
    {
      return fail(path + ": " + error_message);
    }
    std::copy(chunk.begin(), chunk.end(), result.data.begin() + offset);
  }

  if (!soft_reset_ox01f10_sensor(fd, address, error_message)) {
    result.data.clear();
    result.error_message = path + ": " + error_message;
    close(fd);
    return result;
  }
  close(fd);
  result.ok = true;
  return result;
}

}  // namespace

OtpReadResult I2cOtpReader::read(const GmslV4l2CameraConfig & config)
{
  if (config.otp_read_size <= 0) {
    return {false, {}, config.otp_i2c_bus, "otp_read_size must be positive"};
  }
  if (config.otp_i2c_bus < 0) {
    return {
      false,
      {},
      config.otp_i2c_bus,
      "camera.driver.otp_i2c_bus must be configured to read GMSL OTP"};
  }

  std::vector<int> buses;
  buses.push_back(config.otp_i2c_bus);

  OtpReadResult last_result;
  for (const auto bus : buses) {
    auto result = read_otp_from_bus(config, bus);
    if (!result.ok) {
      last_result = std::move(result);
      continue;
    }
    std::string serial_number;
    std::string error_message;
    if (decode_ox01f10_otp(result.data, serial_number, error_message)) {
      return result;
    }
    result.ok = false;
    result.error_message = error_message;
    last_result = std::move(result);
  }
  if (last_result.error_message.empty()) {
    last_result.error_message = "no readable OX01F10 OTP data";
  }
  return last_result;
}

void apply_otp_calibration(
  GmslV4l2CameraConfig & config,
  const GmslV4l2CameraRuntime::OtpReaderFactory & otp_reader_factory)
{
  {
    std::ostringstream trace;
    trace << "otp start read_on_start=" << (config.read_otp_on_start ? "true" : "false") <<
      " factory=" << (otp_reader_factory ? "ready" : "missing") << " bus=" <<
      config.otp_i2c_bus << " address=0x" << std::hex << config.otp_i2c_address <<
      " offset=0x" << config.otp_read_offset << std::dec << " size=" << config.otp_read_size;
    append_trace_line(trace.str());
  }
  if (!config.read_otp_on_start || !otp_reader_factory) {
    append_trace_line("otp skipped");
    return;
  }
  if (config.otp_i2c_bus < 0) {
    config.otp_status = "failed";
    config.otp_result_i2c_bus = config.otp_i2c_bus;
    config.otp_error_message = "camera.driver.otp_i2c_bus must be configured to read GMSL OTP";
    RCLCPP_WARN(
      rclcpp::get_logger("gmsl_v4l2_camera_driver"),
      "failed to read GMSL OTP data: %s",
      config.otp_error_message.c_str());
    append_trace_line("otp failed: " + config.otp_error_message);
    return;
  }

  config.otp_status = "attempted";
  RCLCPP_INFO(
    rclcpp::get_logger("gmsl_v4l2_camera_driver"),
    "reading GMSL OTP data: bus=%d address=0x%x offset=0x%x size=%d",
    config.otp_i2c_bus,
    config.otp_i2c_address,
    config.otp_read_offset,
    config.otp_read_size);
  const auto otp_reader = otp_reader_factory();
  if (!otp_reader) {
    config.otp_status = "failed";
    config.otp_error_message = "otp reader is unavailable";
    append_trace_line("otp failed: " + config.otp_error_message);
    return;
  }

  const auto otp_result = otp_reader->read(config);
  config.otp_result_i2c_bus = otp_result.i2c_bus >= 0 ? otp_result.i2c_bus : config.otp_i2c_bus;
  if (!otp_result.ok) {
    config.otp_status = "failed";
    config.otp_error_message = otp_result.error_message;
    RCLCPP_WARN(
      rclcpp::get_logger("gmsl_v4l2_camera_driver"),
      "failed to read GMSL OTP data: %s",
      otp_result.error_message.c_str());
    append_trace_line("otp failed: " + otp_result.error_message);
    return;
  }

  std::string otp_serial_number;
  std::string otp_error_message;
  const auto otp_calibration = decode_ox01f10_otp(
    otp_result.data,
    otp_serial_number,
    otp_error_message);
  if (!otp_calibration) {
    config.otp_status = "failed";
    config.otp_error_message = otp_error_message;
    RCLCPP_WARN(
      rclcpp::get_logger("gmsl_v4l2_camera_driver"),
      "failed to decode GMSL OTP data from i2c bus %d: %s",
      otp_result.i2c_bus,
      otp_error_message.c_str());
    append_trace_line("otp decode failed: " + otp_error_message);
    return;
  }

  config.serial_number = otp_serial_number;
  config.calibration = otp_calibration;
  config.otp_status = "success";
  config.otp_error_message.clear();
  RCLCPP_INFO(
    rclcpp::get_logger("gmsl_v4l2_camera_driver"),
    "loaded GMSL OTP calibration from i2c bus %d, serial_number=%s",
    otp_result.i2c_bus,
    otp_serial_number.c_str());
  {
    std::ostringstream trace;
    trace << "otp success bus=" << otp_result.i2c_bus << " serial_number=" <<
      otp_serial_number << " width=" << otp_calibration->width << " height=" <<
      otp_calibration->height << " distortion_model=" << otp_calibration->distortion_model <<
      " d_size=" << otp_calibration->d.size();
    append_trace_line(trace.str());
  }
}

GmslV4l2CameraRuntime::GmslV4l2CameraRuntime(
  CaptureBackendFactory capture_factory,
  OtpReaderFactory otp_reader_factory)
: capture_factory_(std::move(capture_factory)),
  otp_reader_factory_(std::move(otp_reader_factory)),
  state_(LifecycleState::kFinalized)
{
  if (!capture_factory_) {
    capture_factory_ = []() {return std::make_unique<V4l2CaptureBackend>();};
  }
  if (!otp_reader_factory_) {
    otp_reader_factory_ = []() {return std::make_unique<I2cOtpReader>();};
  }
}

GmslV4l2CameraRuntime::~GmslV4l2CameraRuntime()
{
  stop_capture_thread();
  if (capture_) {
    capture_->close();
  }
}

void GmslV4l2CameraRuntime::initialize(const device_manager::DeviceDefinition & definition)
{
  std::lock_guard<std::mutex> lock(mutex_);
  device_id_ = definition.id;
}

LifecycleState GmslV4l2CameraRuntime::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

TransitionOutcome GmslV4l2CameraRuntime::materialize(const ParameterMap &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LifecycleState::kFinalized || capture_) {
    return {TransitionResult::kFailure, "runtime is already materialized"};
  }
  capture_ = capture_factory_();
  state_ = LifecycleState::kUnconfigured;
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome GmslV4l2CameraRuntime::dematerialize()
{
  stop_capture_thread();
  std::unique_ptr<CaptureBackend> capture;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!capture_ || state_ == LifecycleState::kFinalized) {
      return {TransitionResult::kFailure, "runtime is not materialized"};
    }
    state_ = LifecycleState::kShuttingDown;
    capture = std::move(capture_);
    reset_ros_output();
    last_connected_.reset();
  }
  capture->close();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = LifecycleState::kFinalized;
  }
  return {TransitionResult::kSuccess, {}};
}

void GmslV4l2CameraRuntime::set_event_handler(device_manager::EventHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  event_handler_ = std::move(handler);
}

void GmslV4l2CameraRuntime::register_hook(device_manager::HookRegistrar registrar)
{
  auto handled_event_revision = std::make_shared<std::uint64_t>(0U);
  registrar(
    [handled_event_revision](const device_manager::HookContext & context) {
      if (!context.enabled) {
        return recovery_requests(context.state, LifecycleState::kFinalized);
      }
      if (context.state == LifecycleState::kFinalized) {
        return std::vector<TransitionRequest>{
          {LifecycleState::kFinalized, LifecycleState::kUnconfigured, context.parameters}};
      }
      if (context.state == LifecycleState::kUnconfigured) {
        return std::vector<TransitionRequest>{
          {LifecycleState::kUnconfigured, LifecycleState::kInactive, context.parameters}};
      }
      if (context.state == LifecycleState::kInactive) {
        return std::vector<TransitionRequest>{
          {LifecycleState::kInactive, LifecycleState::kActive, {}}};
      }
      if (context.state != LifecycleState::kActive || !context.latest_event ||
      !event_needs_recovery(*context.latest_event) ||
      context.latest_event_revision <= *handled_event_revision)
      {
        return std::vector<TransitionRequest>{};
      }

      *handled_event_revision = context.latest_event_revision;
      return recovery_requests(context.state, context.latest_event->target_state);
    });
}

TransitionOutcome GmslV4l2CameraRuntime::configure(const ParameterMap & parameters)
{
  std::string error_message;
  auto config = config_from(parameters, error_message);
  if (!error_message.empty()) {
    return {TransitionResult::kFailure, error_message};
  }

  CaptureBackend * capture = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kUnconfigured || !capture_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    config_ = config;
    state_ = LifecycleState::kConfiguring;
    output_node_ = std::make_shared<rclcpp::Node>(
      config_.camera_name, "/" + config_.camera_name,
      rclcpp::NodeOptions().use_global_arguments(false));
    image_publisher_ = output_node_->create_publisher<sensor_msgs::msg::Image>(
      config_.image_topic, rclcpp::SensorDataQoS());
    if (config_.publish_distorted_image) {
      distorted_image_publisher_ = output_node_->create_publisher<sensor_msgs::msg::Image>(
        config_.distorted_image_topic, rclcpp::SensorDataQoS());
    }
    camera_info_publisher_ = output_node_->create_publisher<sensor_msgs::msg::CameraInfo>(
      config_.camera_info_topic, 10);
    serial_number_publisher_ = output_node_->create_publisher<std_msgs::msg::String>(
      config_.serial_number_topic, 10);
    event_publisher_ = output_node_->create_publisher<device_manager_msgs::msg::DeviceEvent>(
      "~/device_event", 10);
    capture = capture_.get();
  }

  const bool configured = capture->configure(config, error_message);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = configured ? LifecycleState::kInactive : LifecycleState::kUnconfigured;
    if (!configured) {
      reset_ros_output();
      last_connected_.reset();
    }
  }
  if (!configured) {
    emit_connection_event(
      false,
      "gmsl_v4l2.open_failed",
      error_message,
      LifecycleState::kUnconfigured);
    return {TransitionResult::kError, error_message};
  }
  apply_otp_calibration(config, otp_reader_factory_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
    undistort_map_.reset();
  }
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome GmslV4l2CameraRuntime::activate()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kInactive || !capture_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    state_ = LifecycleState::kActive;
  }
  running_.store(true);
  capture_thread_ = std::thread([this]() {capture_loop();});
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome GmslV4l2CameraRuntime::deactivate()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kActive || !capture_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    state_ = LifecycleState::kInactive;
  }
  stop_capture_thread();
  return {TransitionResult::kSuccess, {}};
}

TransitionOutcome GmslV4l2CameraRuntime::cleanup(const ParameterMap &)
{
  stop_capture_thread();
  CaptureBackend * capture = nullptr;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kInactive || !capture_) {
      return {TransitionResult::kFailure, "source state does not match"};
    }
    state_ = LifecycleState::kCleaningUp;
    capture = capture_.get();
    reset_ros_output();
    last_connected_.reset();
  }
  capture->close();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = LifecycleState::kUnconfigured;
  }
  return {TransitionResult::kSuccess, {}};
}

GmslV4l2CameraConfig GmslV4l2CameraRuntime::config_from(
  const ParameterMap & parameters,
  std::string & error_message) const
{
  GmslV4l2CameraConfig config;
  config.camera_name = first_string(
    parameters,
    {"camera.basic.camera_name", "camera.base.camera_name"},
    device_id_.empty() ? config.camera_name : device_id_,
    error_message);
  if (!error_message.empty()) {
    return config;
  }
  config.frame_id = first_string(
    parameters,
    {"camera.basic.frame_id", "camera.basic.color_optical_frame_id"},
    config.camera_name + "_link",
    error_message);
  config.serial_number = first_string(
    parameters,
    {"camera.basic.serial_number", "camera.driver.serial_number"},
    config.serial_number,
    error_message);
  config.video_device = first_string(
    parameters,
    {"device.interface.gmsl_device", "camera.driver.video_device"},
    config.video_device,
    error_message);
  config.image_topic = first_string(
    parameters,
    {"camera.driver.image_topic"},
    config.image_topic,
    error_message);
  config.distorted_image_topic = first_string(
    parameters,
    {"camera.driver.distorted_image_topic"},
    config.distorted_image_topic,
    error_message);
  config.camera_info_topic = first_string(
    parameters,
    {"camera.driver.camera_info_topic"},
    config.camera_info_topic,
    error_message);
  config.serial_number_topic = first_string(
    parameters,
    {"camera.driver.serial_number_topic"},
    config.serial_number_topic,
    error_message);
  config.pixel_format = first_string(
    parameters,
    {"camera.driver.color_format", "camera.driver.pixel_format"},
    config.pixel_format,
    error_message);
  config.intrinsic_params = first_string(
    parameters,
    {"camera.driver.intrinsic_params"},
    config.intrinsic_params,
    error_message);
  if (const auto serial_number = string_field(config.intrinsic_params, "serial_number")) {
    if (config.serial_number.empty()) {
      config.serial_number = *serial_number;
    }
  } else if (const auto sn = string_field(config.intrinsic_params, "sn")) {
    if (config.serial_number.empty()) {
      config.serial_number = *sn;
    }
  }
  config.calibration = parse_camera_calibration(config.intrinsic_params, error_message);
  if (!error_message.empty()) {
    return config;
  }
  config.width = first_int(
    parameters,
    {"camera.driver.color_width", "camera.driver.image_resolution_width"},
    config.width,
    error_message);
  config.height = first_int(
    parameters,
    {"camera.driver.color_height", "camera.driver.image_resolution_height"},
    config.height,
    error_message);
  config.fps = first_int(
    parameters,
    {"camera.driver.color_fps", "camera.driver.fps"},
    config.fps,
    error_message);
  config.read_timeout_ms = first_int(
    parameters,
    {"camera.driver.read_timeout_ms"},
    config.read_timeout_ms,
    error_message);
  config.undistort_image = first_bool(
    parameters,
    {"camera.driver.undistort_image"},
    config.undistort_image,
    error_message);
  if (!error_message.empty()) {
    return config;
  }
  config.undistort_focal_scale = first_double(
    parameters,
    {"camera.driver.undistort_focal_scale"},
    config.undistort_focal_scale,
    error_message);
  if (!error_message.empty()) {
    return config;
  }
  config.calibration_crop_y = first_number(
    parameters,
    {"camera.driver.calibration_crop_y"},
    config.calibration_crop_y,
    error_message);
  if (!error_message.empty()) {
    return config;
  }
  config.rotate_180 = first_bool(
    parameters,
    {"camera.driver.rotate_180"},
    config.rotate_180,
    error_message);
  if (!error_message.empty()) {
    return config;
  }
  config.publish_distorted_image = first_bool(
    parameters,
    {"camera.driver.publish_distorted_image"},
    config.publish_distorted_image,
    error_message);
  if (!error_message.empty()) {
    return config;
  }
  config.read_otp_on_start = first_bool(
    parameters,
    {"camera.driver.read_otp_on_start"},
    config.read_otp_on_start,
    error_message);
  if (!error_message.empty()) {
    return config;
  }
  config.otp_i2c_bus = first_number(
    parameters,
    {"camera.driver.otp_i2c_bus"},
    config.otp_i2c_bus,
    error_message);
  if (!error_message.empty()) {
    return config;
  }
  config.otp_i2c_address = first_number(
    parameters,
    {"camera.driver.otp_i2c_address"},
    config.otp_i2c_address,
    error_message);
  if (!error_message.empty()) {
    return config;
  }
  config.otp_read_offset = first_number(
    parameters,
    {"camera.driver.otp_read_offset"},
    config.otp_read_offset,
    error_message);
  if (!error_message.empty()) {
    return config;
  }
  config.otp_read_size = first_number(
    parameters,
    {"camera.driver.otp_read_size"},
    config.otp_read_size,
    error_message);
  if (!error_message.empty()) {
    return config;
  }
  return config;
}

void GmslV4l2CameraRuntime::capture_loop()
{
  auto next_publish_at = std::chrono::steady_clock::now();
  int publish_fps = 0;
  while (running_.load()) {
    CaptureBackend * capture = nullptr;
    GmslV4l2CameraConfig config;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      capture = capture_.get();
      config = config_;
    }
    if (capture == nullptr) {
      return;
    }
    if (config.fps != publish_fps) {
      publish_fps = std::max(config.fps, 1);
      next_publish_at = std::chrono::steady_clock::now();
    }

    std::string error_message;
    const auto frame = capture->read_frame(
      std::chrono::milliseconds(config.read_timeout_ms),
      error_message);
    if (!running_.load()) {
      return;
    }
    if (frame) {
      emit_connection_event(
        true,
        "gmsl_v4l2.online",
        "GMSL V4L2 camera online",
        LifecycleState::kActive);
      const auto now = std::chrono::steady_clock::now();
      if (now < next_publish_at) {
        continue;
      }
      publish_frame(*frame);
      const auto publish_period = std::chrono::nanoseconds(std::chrono::seconds(1)) / publish_fps;
      do {
        next_publish_at += publish_period;
      } while (next_publish_at <= now);
      continue;
    }
    if (!error_message.empty()) {
      emit_connection_event(
        false,
        "gmsl_v4l2.capture_failed",
        error_message,
        LifecycleState::kUnconfigured);
    }
  }
}

void GmslV4l2CameraRuntime::publish_frame(const CapturedFrame & frame)
{
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr distorted_image_publisher;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_number_publisher;
  std::shared_ptr<rclcpp::Node> node;
  GmslV4l2CameraConfig config;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    image_publisher = image_publisher_;
    distorted_image_publisher = distorted_image_publisher_;
    info_publisher = camera_info_publisher_;
    serial_number_publisher = serial_number_publisher_;
    node = output_node_;
    config = config_;
  }
  if (!node || !image_publisher || !info_publisher || !serial_number_publisher) {
    return;
  }

  bool should_log_first_frame = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    should_log_first_frame = !logged_first_frame_;
    if (should_log_first_frame) {
      logged_first_frame_ = true;
    }
  }
  const auto frame_start = std::chrono::steady_clock::now();
  if (should_log_first_frame) {
    std::ostringstream trace;
    trace << "received width=" << frame.width << " height=" << frame.height << " format=" <<
      frame.pixel_format << " bytes=" << frame.data.size() << " undistort=" <<
      (config.undistort_image ? "true" : "false") << " publish_distorted=" <<
      (config.publish_distorted_image ? "true" : "false");
    append_trace_line(trace.str());
    RCLCPP_INFO(
      node->get_logger(),
      "first GMSL frame received: width=%d height=%d format=%s bytes=%zu "
      "undistort=%s publish_distorted=%s",
      frame.width,
      frame.height,
      frame.pixel_format.c_str(),
      frame.data.size(),
      config.undistort_image ? "true" : "false",
      config.publish_distorted_image ? "true" : "false");
  }

  const auto bgr = convert_to_bgr8(frame);
  if (should_log_first_frame) {
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - frame_start);
    RCLCPP_INFO(
      node->get_logger(),
      "first GMSL frame converted: bgr_bytes=%zu elapsed_ms=%ld",
      bgr.size(),
      elapsed.count());
    std::ostringstream trace;
    trace << "converted bgr_bytes=" << bgr.size() << " elapsed_ms=" << elapsed.count();
    append_trace_line(trace.str());
  }
  if (bgr.size() != static_cast<std::size_t>(frame.width) * frame.height * 3U) {
    std::ostringstream trace;
    trace << "unsupported_format expected=" <<
      (static_cast<std::size_t>(frame.width) * frame.height * 3U) << " actual=" << bgr.size();
    append_trace_line(trace.str());
    emit_connection_event(
      false,
      "gmsl_v4l2.unsupported_format",
      "unsupported or malformed frame format: " + frame.pixel_format,
      LifecycleState::kUnconfigured);
    return;
  }

  auto stamp = node->now();
  std::vector<std::uint8_t> image_data = bgr;
  int image_width = frame.width;
  int image_height = frame.height;
  std::optional<CameraCalibration> camera_info_calibration;
  std::optional<CameraCalibration> capture_calibration;
  if (config.calibration) {
    capture_calibration = scale_calibration_to_frame(
      *config.calibration, frame.width, frame.height, config.calibration_crop_y);
    camera_info_calibration = capture_calibration;
    if (should_log_first_frame &&
      (capture_calibration->width != config.calibration->width ||
      capture_calibration->height != config.calibration->height))
    {
      std::ostringstream trace;
      trace << "scaled_calibration from=" << config.calibration->width << "x" <<
        config.calibration->height << " to=" << capture_calibration->width << "x" <<
        capture_calibration->height << " fx=" << capture_calibration->k[0] << " fy=" <<
        capture_calibration->k[4] << " cx=" << capture_calibration->k[2] << " cy=" <<
        capture_calibration->k[5] << " configured_crop_y=" << config.calibration_crop_y;
      append_trace_line(trace.str());
    }
  }

  bool image_undistorted = false;
  if (config.undistort_image && capture_calibration &&
    supports_undistortion(*capture_calibration))
  {
    const auto undistort_start = std::chrono::steady_clock::now();
    auto output_calibration = wide_fov_output_calibration(
      *capture_calibration,
      config.undistort_focal_scale);
    const auto map = undistort_map_for(
      *capture_calibration,
      output_calibration,
      frame.width,
      frame.height);
    if (map) {
      image_data = remap_bgr8(bgr, frame.width, frame.height, *map);
      if (image_data.size() == static_cast<std::size_t>(frame.width) * frame.height * 3U) {
        image_width = frame.width;
        image_height = frame.height;
        camera_info_calibration = output_calibration;
        image_undistorted = true;
      } else {
        image_data = bgr;
      }
    }
    if (should_log_first_frame) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - undistort_start);
      RCLCPP_INFO(
        node->get_logger(),
        "first GMSL frame undistort step: enabled=%s map=%s elapsed_ms=%ld",
        config.undistort_image ? "true" : "false",
        map ? "ready" : "unavailable",
        elapsed.count());
      std::ostringstream trace;
      trace << "undistort map=" << (map ? "ready" : "unavailable") << " elapsed_ms=" <<
        elapsed.count();
      append_trace_line(trace.str());
      if (image_undistorted) {
        std::ostringstream output_trace;
        output_trace << "undistort_output from=" << frame.width << "x" << frame.height <<
          " to=" << image_width << "x" << image_height << " focal_scale=" <<
          config.undistort_focal_scale << " fx=" << camera_info_calibration->k[0] <<
          " fy=" << camera_info_calibration->k[4];
        append_trace_line(output_trace.str());
      }
    }
  }

  if (config.publish_distorted_image && distorted_image_publisher) {
    sensor_msgs::msg::Image distorted_image;
    distorted_image.header.stamp = stamp;
    distorted_image.header.frame_id = config.frame_id;
    distorted_image.height = static_cast<std::uint32_t>(frame.height);
    distorted_image.width = static_cast<std::uint32_t>(frame.width);
    distorted_image.encoding = "bgr8";
    distorted_image.is_bigendian = false;
    distorted_image.step = static_cast<std::uint32_t>(frame.width * 3);
    distorted_image.data = bgr;
    distorted_image_publisher->publish(distorted_image);
  }

  if (config.rotate_180) {
    cv::Mat rotated(
      image_height, image_width, CV_8UC3, image_data.data());
    cv::rotate(rotated, rotated, cv::ROTATE_180);
    if (camera_info_calibration) {
      camera_info_calibration->k[2] = static_cast<double>(image_width - 1) -
        camera_info_calibration->k[2];
      camera_info_calibration->k[5] = static_cast<double>(image_height - 1) -
        camera_info_calibration->k[5];
      camera_info_calibration->p[2] = camera_info_calibration->k[2];
      camera_info_calibration->p[6] = camera_info_calibration->k[5];
    }
  }

  sensor_msgs::msg::Image image;
  image.header.stamp = stamp;
  image.header.frame_id = config.frame_id;
  image.height = static_cast<std::uint32_t>(image_height);
  image.width = static_cast<std::uint32_t>(image_width);
  image.encoding = "bgr8";
  image.is_bigendian = false;
  image.step = static_cast<std::uint32_t>(image_width * 3);
  image.data = std::move(image_data);

  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header = image.header;
  camera_info.height = image.height;
  camera_info.width = image.width;
  if (camera_info_calibration) {
    camera_info.distortion_model = camera_info_calibration->distortion_model;
    camera_info.d = camera_info_calibration->d;
    camera_info.k = camera_info_calibration->k;
    camera_info.r = camera_info_calibration->r;
    camera_info.p = camera_info_calibration->p;
    if (image_undistorted) {
      set_zero_distortion(camera_info);
    }
  }

  if (!config.serial_number.empty()) {
    std_msgs::msg::String serial_number;
    serial_number.data = config.serial_number;
    serial_number_publisher->publish(serial_number);
  }

  image_publisher->publish(image);
  info_publisher->publish(camera_info);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!logged_first_publish_) {
      logged_first_publish_ = true;
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - frame_start);
      RCLCPP_INFO(
        node->get_logger(),
        "first GMSL frame published: topic=%s camera_info_topic=%s width=%u height=%u "
        "encoding=%s camera_info_distortion_model=%s distortion_size=%zu elapsed_ms=%ld",
        config.image_topic.c_str(),
        config.camera_info_topic.c_str(),
        image.width,
        image.height,
        image.encoding.c_str(),
        camera_info.distortion_model.c_str(),
        camera_info.d.size(),
        elapsed.count());
      std::ostringstream trace;
      trace << "published image_topic=" << config.image_topic << " info_topic=" <<
        config.camera_info_topic << " width=" << image.width << " height=" << image.height <<
        " encoding=" << image.encoding << " distortion_size=" << camera_info.d.size() <<
        " elapsed_ms=" << elapsed.count();
      append_trace_line(trace.str());
    }
  }
}

std::shared_ptr<const UndistortMap> GmslV4l2CameraRuntime::undistort_map_for(
  const CameraCalibration & source_calibration,
  const CameraCalibration & output_calibration,
  int width,
  int height)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (undistort_map_ &&
    calibration_matches_map(*undistort_map_, source_calibration, output_calibration, width, height))
  {
    return undistort_map_;
  }
  undistort_map_ = build_undistort_map(source_calibration, output_calibration, width, height);
  return undistort_map_;
}

void GmslV4l2CameraRuntime::emit_event(device_manager::DeviceEvent event)
{
  device_manager::EventHandler handler;
  rclcpp::Publisher<device_manager_msgs::msg::DeviceEvent>::SharedPtr event_publisher;
  std::shared_ptr<rclcpp::Node> node;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handler = event_handler_;
    event_publisher = event_publisher_;
    node = output_node_;
  }
  if (event_publisher && node) {
    event_publisher->publish(to_ros_event(event, node->now()));
  }
  if (handler) {
    handler(std::move(event));
  }
}

void GmslV4l2CameraRuntime::emit_connection_event(
  bool connected,
  const std::string & code,
  const std::string & message,
  LifecycleState target_state)
{
  std::string video_device;
  std::string serial_number;
  bool otp_read_on_start {false};
  std::string otp_status;
  int otp_i2c_bus {-1};
  std::string otp_error_message;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_connected_ && *last_connected_ == connected) {
      return;
    }
    last_connected_ = connected;
    video_device = config_.video_device;
    serial_number = config_.serial_number;
    otp_read_on_start = config_.read_otp_on_start;
    otp_status = config_.otp_status;
    otp_i2c_bus = config_.otp_result_i2c_bus;
    otp_error_message = config_.otp_error_message;
  }

  device_manager::DeviceEvent event;
  event.timestamp = device_manager::Clock::now();
  event.level = connected ? device_manager::EventLevel::kOk : device_manager::EventLevel::kError;
  event.code = code;
  event.message = message;
  event.source = "gmsl_v4l2_camera_driver";
  event.target_state = target_state;
  event.values["video_device"] = video_device;
  event.values["otp_read_on_start"] = otp_read_on_start ? "true" : "false";
  event.values["otp_status"] = otp_status;
  event.values["otp_i2c_bus"] = std::to_string(otp_i2c_bus);
  if (!otp_error_message.empty()) {
    event.values["otp_error_message"] = otp_error_message;
  }
  if (!serial_number.empty()) {
    event.values["serial_number"] = serial_number;
  }
  emit_event(std::move(event));
}

void GmslV4l2CameraRuntime::stop_capture_thread()
{
  running_.store(false);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

void GmslV4l2CameraRuntime::reset_ros_output()
{
  event_publisher_.reset();
  serial_number_publisher_.reset();
  camera_info_publisher_.reset();
  distorted_image_publisher_.reset();
  image_publisher_.reset();
  undistort_map_.reset();
  logged_first_frame_ = false;
  logged_first_publish_ = false;
  output_node_.reset();
}

}  // namespace gmsl_v4l2_camera_driver

PLUGINLIB_EXPORT_CLASS(
  gmsl_v4l2_camera_driver::GmslV4l2CameraRuntime,
  device_manager::IDeviceRuntime)
