#include <chrono>
#include <csignal>
#include <fstream>
#include <memory>
#include <string>

#include <unistd.h>

#include "device_manager_msgs/msg/device_event.hpp"
#include "device_manager_msgs/msg/device_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("device_manager_ros_test_topic_source");
  const auto topic = node->declare_parameter<std::string>("topic", "/process_runtime_ready");
  const auto publish_event = node->declare_parameter<bool>("publish_event", false);
  const auto exit_after_ms = node->declare_parameter<int>("exit_after_ms", 0);
  const auto empty_string = node->declare_parameter<std::string>("empty_string", "default");
  const auto numeric_string = node->declare_parameter<std::string>("numeric_string", "default");
  const auto unmapped = node->declare_parameter<std::string>("unmapped", "default");
  const auto pid_file = node->declare_parameter<std::string>("pid_file", "");
  const auto child_pid_file = node->declare_parameter<std::string>("child_pid_file", "");
  if (!pid_file.empty()) {
    std::ofstream(pid_file) << getpid();
  }
  if (!child_pid_file.empty()) {
    const auto child_pid = fork();
    if (child_pid == 0) {
      std::signal(SIGTERM, SIG_IGN);
      std::ofstream(child_pid_file) << getpid();
      while (true) {
        pause();
      }
    }
  }
  auto publisher = node->create_publisher<std_msgs::msg::String>(topic, rclcpp::SensorDataQoS());
  auto event_publisher = node->create_publisher<device_manager_msgs::msg::DeviceEvent>(
    "~/device_event", rclcpp::QoS(10));
  auto timer = node->create_wall_timer(
    20ms,
    [publisher, event_publisher, publish_event, empty_string, numeric_string, unmapped]() {
      std_msgs::msg::String message;
      message.data = "ready";
      publisher->publish(message);

      if (!publish_event) {
        return;
      }
      device_manager_msgs::msg::DeviceEvent event;
      event.level = device_manager_msgs::msg::DeviceEvent::ERROR;
      event.code = "test.driver_fault";
      event.message = "driver reported a test fault";
      event.source = "test_topic_source";
      event.target_state = device_manager_msgs::msg::DeviceState::UNCONFIGURED;
      diagnostic_msgs::msg::KeyValue empty_value;
      empty_value.key = "empty_string";
      empty_value.value = empty_string;
      event.values.push_back(empty_value);
      diagnostic_msgs::msg::KeyValue numeric_value;
      numeric_value.key = "numeric_string";
      numeric_value.value = numeric_string;
      event.values.push_back(numeric_value);
      diagnostic_msgs::msg::KeyValue unmapped_value;
      unmapped_value.key = "unmapped";
      unmapped_value.value = unmapped;
      event.values.push_back(unmapped_value);
      event_publisher->publish(event);
    });
  rclcpp::TimerBase::SharedPtr exit_timer;
  if (exit_after_ms > 0) {
    exit_timer = node->create_wall_timer(
      std::chrono::milliseconds(exit_after_ms), [node]() {
        node->get_node_base_interface()->get_context()->shutdown("test process exit");
      });
  }
  (void)timer;
  (void)exit_timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
