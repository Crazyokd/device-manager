#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "device_manager_core/device_manager_api.hpp"
#include "device_manager_msgs/msg/device_state_array.hpp"
#include "device_manager_msgs/srv/change_device_state.hpp"
#include "device_manager_msgs/srv/get_devices.hpp"
#include "device_manager_msgs/srv/patch_device_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

namespace device_manager_ros
{

class Ros2DeviceManagerApi final : public device_manager::DeviceManagerApi
{
public:
  Ros2DeviceManagerApi(
    device_manager::IDeviceManagerApplication & application, rclcpp::Node & node,
    std::chrono::milliseconds publish_period, std::string api_prefix = "~");
  ~Ros2DeviceManagerApi() override;

  Ros2DeviceManagerApi(const Ros2DeviceManagerApi &) = delete;
  Ros2DeviceManagerApi & operator=(const Ros2DeviceManagerApi &) = delete;

  void start() override;
  void stop() override;

private:
  using ChangeDeviceState = device_manager_msgs::srv::ChangeDeviceState;
  using GetDevices = device_manager_msgs::srv::GetDevices;
  using PatchDeviceParameters = device_manager_msgs::srv::PatchDeviceParameters;

  device_manager_msgs::msg::DeviceStateArray snapshot_message();
  void publish_snapshot();
  void handle_change_state(
    const std::shared_ptr<ChangeDeviceState::Request> request,
    std::shared_ptr<ChangeDeviceState::Response> response);
  void handle_patch_parameters(
    const std::shared_ptr<PatchDeviceParameters::Request> request,
    std::shared_ptr<PatchDeviceParameters::Response> response);

  rclcpp::Node & node_;
  std::chrono::milliseconds publish_period_;
  std::string api_prefix_;
  rclcpp::Publisher<device_manager_msgs::msg::DeviceStateArray>::SharedPtr publisher_;
  rclcpp::Service<GetDevices>::SharedPtr get_devices_service_;
  rclcpp::Service<ChangeDeviceState>::SharedPtr change_state_service_;
  rclcpp::Service<PatchDeviceParameters>::SharedPtr patch_parameters_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace device_manager_ros
