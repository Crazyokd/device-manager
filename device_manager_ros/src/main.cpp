#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "device_manager_core/device_manager_application.hpp"
#include "device_manager_ros/device_registration.hpp"
#include "device_manager_ros/ros2_device_manager_api.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace
{

class DeviceManagerNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  DeviceManagerNode()
  : LifecycleNode("device_manager"), runtime_node_(std::make_shared<rclcpp::Node>(
        "device_manager_runtime", rclcpp::NodeOptions().use_global_arguments(false)))
  {
    declare_parameter<std::string>("config_file", "");
    declare_parameter<std::string>(
      "parameter_api_url", "http://127.0.0.1:3000/parameters");
    declare_parameter<int>("update_period_ms", 1000);
    declare_parameter<int>("service_timeout_ms", 1000);
  }

  std::shared_ptr<rclcpp::Node> runtime_node() const
  {
    return runtime_node_;
  }

private:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    try {
      const auto update_period_ms = get_parameter("update_period_ms").as_int();
      const auto service_timeout_ms = get_parameter("service_timeout_ms").as_int();
      if (update_period_ms <= 0 || service_timeout_ms <= 0) {
        throw std::invalid_argument("update_period_ms and service_timeout_ms must be positive");
      }
      const auto timeout = std::chrono::milliseconds(service_timeout_ms);
      const auto api_url = get_parameter("parameter_api_url").as_string();
      auto registrations = device_manager_ros::load_device_registrations(
        *runtime_node_, get_parameter("config_file").as_string(), timeout,
        [&api_url, timeout](const std::string & id) {
          return device_manager_ros::load_device_parameters(api_url, id, timeout);
        });
      manager_ = std::make_unique<device_manager::DeviceManager>(std::move(registrations));
      application_ = std::make_unique<device_manager::DeviceManagerApplication>(
        *manager_, std::chrono::milliseconds(update_period_ms));
      api_ = std::make_unique<device_manager_ros::Ros2DeviceManagerApi>(
        *application_, *runtime_node_, std::chrono::milliseconds(update_period_ms),
        "/device_manager");
      return CallbackReturn::SUCCESS;
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(get_logger(), "failed to configure device manager: %s", exception.what());
      reset();
      return CallbackReturn::FAILURE;
    }
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (!application_ || !api_) {
      return CallbackReturn::FAILURE;
    }
    application_->start();
    api_->start();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    stop();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    reset();
    return CallbackReturn::SUCCESS;
  }

  void stop()
  {
    if (api_) {
      api_->stop();
    }
    if (application_) {
      application_->stop();
    }
  }

  void reset()
  {
    stop();
    api_.reset();
    application_.reset();
    manager_.reset();
  }

  std::shared_ptr<rclcpp::Node> runtime_node_;
  std::unique_ptr<device_manager::DeviceManager> manager_;
  std::unique_ptr<device_manager::DeviceManagerApplication> application_;
  std::unique_ptr<device_manager_ros::Ros2DeviceManagerApi> api_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeviceManagerNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node->get_node_base_interface());
  executor.add_node(node->runtime_node());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
