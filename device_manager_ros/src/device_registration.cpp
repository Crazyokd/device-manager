#include "device_manager_ros/device_registration.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "curl/curl.h"
#include "device_manager_ros/ros2_lifecycle_driver_adapter.hpp"
#include "device_manager_ros/ros2_process_device_runtime_adapter.hpp"
#include "pluginlib/class_loader.hpp"
#include "yaml-cpp/yaml.h"

namespace device_manager_ros
{
namespace
{

using device_manager::ParameterMap;
using device_manager::ParameterValue;

pluginlib::ClassLoader<device_manager::IDeviceRuntime> & runtime_plugin_loader()
{
  static pluginlib::ClassLoader<device_manager::IDeviceRuntime> loader{
    "device_manager_core", "device_manager::IDeviceRuntime"};
  return loader;
}

ParameterValue parameter_value(const YAML::Node & item)
{
  const auto type = item["type"].as<std::string>();
  const auto value = item["value"];
  if (type == "bool") {return value.as<bool>();}
  if (type == "int32" || type == "int64") {return value.as<std::int64_t>();}
  if (type == "float32" || type == "float64") {return value.as<double>();}
  if (type == "string" || type == "enum") {return value.as<std::string>();}
  throw std::runtime_error("unsupported device parameter type: " + type);
}

bool enabled_from(const ParameterMap & parameters)
{
  const auto found = parameters.find("device.enable");
  if (found == parameters.end()) {
    return true;
  }
  if (!std::holds_alternative<bool>(found->second)) {
    throw std::runtime_error("device.enable must be a boolean");
  }
  return std::get<bool>(found->second);
}

ParameterMap topology_parameters_from(const YAML::Node & instance)
{
  ParameterMap result;
  const auto parameters = instance["topology_parameters"];
  if (!parameters) {
    return result;
  }
  if (!parameters.IsMap()) {
    throw std::runtime_error("topology_parameters must be a map");
  }
  for (const auto & parameter : parameters) {
    result.emplace(parameter.first.as<std::string>(), parameter_value(parameter.second));
  }
  return result;
}

Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process_spec_from(
  const YAML::Node & instance)
{
  const auto process = instance["process"];
  if (!process || !process.IsMap()) {
    throw std::runtime_error("ros2_process runtime requires process");
  }

  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec result;
  result.package = process["package"].as<std::string>();
  result.executable = process["executable"].as<std::string>();
  result.node_name = process["node_name"] ?
    process["node_name"].as<std::string>() : instance["device_id"].as<std::string>();
  result.namespace_ = process["namespace"] ?
    process["namespace"].as<std::string>() : "";
  if (const auto mappings = process["parameter_mappings"]) {
    if (!mappings.IsMap()) {
      throw std::runtime_error("process.parameter_mappings must be a map");
    }
    for (const auto & mapping : mappings) {
      result.parameter_mappings.emplace(
        mapping.first.as<std::string>(), mapping.second.as<std::string>());
    }
  }
  if (const auto parameters = process["topology_parameters"]) {
    if (!parameters.IsMap()) {
      throw std::runtime_error("process.topology_parameters must be a map");
    }
    for (const auto & parameter : parameters) {
      result.topology_parameters.emplace(
        parameter.first.as<std::string>(), parameter_value(parameter.second));
    }
  }
  if (process["fixed_parameters"]) {
    throw std::runtime_error(
            "process.fixed_parameters is not supported; parameters must come from the parameter API");
  }
  return result;
}

std::size_t append_response(char * data, std::size_t size, std::size_t count, void * output)
{
  static_cast<std::string *>(output)->append(data, size * count);
  return size * count;
}

}  // namespace

device_manager::ParameterMap parse_device_parameters_response(const std::string & response)
{
  const auto items = YAML::Load(response)["data"]["items"];
  if (!items || !items.IsMap()) {
    throw std::runtime_error("parameter API response must contain data.items");
  }
  ParameterMap result;
  for (const auto & entry : items) {
    if (!entry.second["value"]) {
      continue;
    }
    const auto name = entry.first.as<std::string>();
    result.insert_or_assign(name, parameter_value(entry.second));
  }
  return result;
}

device_manager::ParameterMap load_device_parameters(
  const std::string & api_url, const std::string & device_id,
  std::chrono::milliseconds timeout)
{
  if (api_url.empty() || timeout <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("parameter API URL and timeout must be valid");
  }
  static const bool curl_initialized = []() {
      if (curl_global_init(CURL_GLOBAL_DEFAULT) != CURLE_OK) {
        throw std::runtime_error("failed to initialize libcurl");
      }
      return true;
    }();
  (void)curl_initialized;

  std::unique_ptr<CURL, decltype(& curl_easy_cleanup)> curl(curl_easy_init(), curl_easy_cleanup);
  if (!curl) {
    throw std::runtime_error("failed to create HTTP client");
  }
  std::unique_ptr<char, decltype(& curl_free)> escaped(
    curl_easy_escape(curl.get(), device_id.c_str(), static_cast<int>(device_id.size())), curl_free);
  if (!escaped) {
    throw std::runtime_error("failed to encode device id");
  }
  const auto separator = api_url.find('?') == std::string::npos ? '?' : '&';
  const auto url = api_url + separator + "scope=device&device_id=" + escaped.get();
  std::string response;
  curl_easy_setopt(curl.get(), CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl.get(), CURLOPT_TIMEOUT_MS, static_cast<long>(timeout.count()));
  curl_easy_setopt(curl.get(), CURLOPT_FAILONERROR, 1L);
  curl_easy_setopt(curl.get(), CURLOPT_WRITEFUNCTION, append_response);
  curl_easy_setopt(curl.get(), CURLOPT_WRITEDATA, &response);
  const auto result = curl_easy_perform(curl.get());
  if (result != CURLE_OK) {
    throw std::runtime_error(
            "failed to load parameters for " + device_id + ": " + curl_easy_strerror(result));
  }
  return parse_device_parameters_response(response);
}

std::vector<device_manager::DeviceRegistration> load_device_registrations(
  rclcpp::Node & node, const std::string & config_file,
  std::chrono::milliseconds service_timeout,
  const DeviceParameterProvider & parameter_provider)
{
  if (config_file.empty()) {
    return {};
  }
  const auto root = YAML::LoadFile(config_file);
  const auto instances = root["instances"];
  if (!instances || !instances.IsSequence()) {
    throw std::runtime_error("device config must contain an instances sequence");
  }

  std::vector<device_manager::DeviceRegistration> registrations;
  registrations.reserve(instances.size());
  for (const auto & instance : instances) {
    if (instance["values"] || (instance["process"] && instance["process"]["parameters"])) {
      throw std::runtime_error("device parameters must come from the parameter API");
    }
    const auto id = instance["device_id"].as<std::string>();
    const auto type = instance["device_type"].as<std::string>();
    const auto runtime = instance["runtime"] ?
      instance["runtime"].as<std::string>() : "ros2_lifecycle";
    auto parameters = parameter_provider(id);
    for (auto && [name, value] : topology_parameters_from(instance)) {
      parameters.insert_or_assign(std::move(name), std::move(value));
    }
    const auto enabled = enabled_from(parameters);

    std::shared_ptr<device_manager::IDeviceRuntime> device_runtime;
    if (runtime == "ros2_lifecycle") {
      const auto node_name = instance["node_name"] ?
        instance["node_name"].as<std::string>() : id;
      device_runtime = std::make_unique<Ros2LifecycleDriverAdapter>(
        node, node_name, service_timeout);
    } else if (runtime == "ros2_process") {
      device_runtime = std::make_unique<Ros2ProcessDeviceRuntimeAdapter>(
        node, process_spec_from(instance), service_timeout);
    } else {
      device_runtime = runtime_plugin_loader().createSharedInstance(runtime);
    }
    device_manager::DeviceDefinition definition{id, type, enabled, std::move(parameters)};
    registrations.push_back({std::move(definition), std::move(device_runtime)});
  }
  return registrations;
}

}  // namespace device_manager_ros
