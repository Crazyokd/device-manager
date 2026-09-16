#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "device_manager_core/device_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace device_manager_ros
{

using DeviceParameterProvider =
  std::function<device_manager::ParameterMap(const std::string & device_id)>;

device_manager::ParameterMap load_device_parameters(
  const std::string & api_url, const std::string & device_id,
  std::chrono::milliseconds timeout);

device_manager::ParameterMap parse_device_parameters_response(const std::string & response);

std::vector<device_manager::DeviceRegistration> load_device_registrations(
  rclcpp::Node & node, const std::string & config_file,
  std::chrono::milliseconds service_timeout,
  const DeviceParameterProvider & parameter_provider);

}  // namespace device_manager_ros
