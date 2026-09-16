#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "device_manager_msgs/msg/device_state.hpp"
#include "device_manager_msgs/msg/device_state_array.hpp"
#include "device_manager_msgs/msg/device_event.hpp"
#include "device_manager_msgs/msg/transition_request.hpp"
#include "device_manager_msgs/srv/change_device_state.hpp"
#include "device_manager_msgs/srv/get_devices.hpp"
#include "device_manager_msgs/srv/patch_device_parameters.hpp"
#include "device_manager_core/device_manager_application.hpp"
#include "device_manager_ros/device_registration.hpp"
#include "device_manager_ros/ros2_lifecycle_driver_adapter.hpp"
#include "device_manager_ros/ros2_process_device_runtime_adapter.hpp"
#include "device_manager_ros/ros2_device_manager_api.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

namespace device_manager_ros
{
namespace
{

using namespace std::chrono_literals;

template<typename Predicate>
bool wait_until(Predicate predicate, std::chrono::milliseconds timeout = 2s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(5ms);
  }
  return predicate();
}

class RosFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

class ExecutorThread
{
public:
  explicit ExecutorThread(rclcpp::Executor & executor)
  : executor_(executor), thread_([this]() {executor_.spin();})
  {
  }

  ~ExecutorThread()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

private:
  rclcpp::Executor & executor_;
  std::thread thread_;
};

lifecycle_msgs::msg::State ros_state(std::uint8_t id)
{
  lifecycle_msgs::msg::State state;
  state.id = id;
  return state;
}

TEST_F(RosFixture, LifecycleAdapterMapsStateTransitionsAndFailures)
{
  auto lifecycle_node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("adapter_target");
  lifecycle_node->declare_parameter<double>("gain", 0.0);
  auto client_node = std::make_shared<rclcpp::Node>("adapter_test_client");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(lifecycle_node->get_node_base_interface());
  executor.add_node(client_node);
  Ros2LifecycleDriverAdapter adapter(*client_node, "/adapter_target", 500ms);
  ExecutorThread spin_thread(executor);
  ASSERT_TRUE(wait_until([&adapter]() {
      return adapter.state() == device_manager::LifecycleState::kUnconfigured;
  }));

  const auto configured = adapter.request_transition(
    {device_manager::LifecycleState::kUnconfigured,
      device_manager::LifecycleState::kInactive, {{"gain", 2.0}}});
  EXPECT_EQ(configured.result, device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(adapter.state(), device_manager::LifecycleState::kInactive);
  EXPECT_DOUBLE_EQ(lifecycle_node->get_parameter("gain").as_double(), 2.0);

  const auto unsupported = adapter.request_transition(
    {device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kInactive, {}});
  EXPECT_EQ(unsupported.result, device_manager::TransitionResult::kFailure);
}

TEST_F(RosFixture, Ros2ProcessDeviceRuntimeStartsAndStopsAChildProcess)
{
  auto client_node = std::make_shared<rclcpp::Node>("process_runtime_client");
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto topic = "/process_runtime_ready_" + suffix;

  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "managed_topic_source_" + suffix;

  Ros2ProcessDeviceRuntimeAdapter adapter(
    *client_node, process, 1s);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);

  EXPECT_EQ(adapter.state(), device_manager::LifecycleState::kFinalized);
  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured, {{"topic", topic}}}).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(adapter.state(), device_manager::LifecycleState::kUnconfigured);
  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive, {}}).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive, {}}).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(adapter.state(), device_manager::LifecycleState::kActive);

  EXPECT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kActive,
        device_manager::LifecycleState::kFinalized, {}}).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(adapter.state(), device_manager::LifecycleState::kFinalized);
}

TEST_F(RosFixture, ApplicationStopFinalizesManagedProcess)
{
  auto client_node = std::make_shared<rclcpp::Node>("managed_process_shutdown_client");
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto topic = "/managed_process_shutdown_" + suffix;
  const auto pid_file = std::filesystem::temp_directory_path() /
    ("device_manager_application_stop_" + suffix + ".pid");

  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "managed_process_shutdown_" + suffix;
  process.parameter_mappings = {{"topic", "topic"}, {"pid_file", "pid_file"}};

  std::vector<device_manager::DeviceRegistration> registrations;
  registrations.push_back({
        {"managed_process", "test", true, {{"topic", topic}, {"pid_file", pid_file.string()}}},
        std::make_unique<Ros2ProcessDeviceRuntimeAdapter>(*client_node, process, 1s)});
  device_manager::DeviceManager manager(std::move(registrations));
  device_manager::DeviceManagerApplication application(manager, 5ms);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);
  application.start();

  ASSERT_TRUE(wait_until([&manager, &client_node, &topic]() {
      return manager.device("managed_process")->state ==
             device_manager::LifecycleState::kActive &&
             client_node->count_publishers(topic) > 0U;
    }));
  std::ifstream pid_stream(pid_file);
  pid_t pid = -1;
  pid_stream >> pid;
  ASSERT_GT(pid, 0);
  const auto process_group = getpgid(pid);
  ASSERT_GT(process_group, 0);
  application.stop();

  EXPECT_EQ(
    manager.device("managed_process")->state,
    device_manager::LifecycleState::kFinalized);
  EXPECT_NE(kill(-process_group, 0), 0);
  EXPECT_EQ(errno, ESRCH);
  std::filesystem::remove(pid_file);
}

TEST_F(RosFixture, FinalizeWaitsForManagedProcessGroup)
{
  auto client_node = std::make_shared<rclcpp::Node>("managed_process_group_client");
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto pid_file = std::filesystem::temp_directory_path() /
    ("device_manager_process_group_" + suffix + ".pid");

  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "managed_process_group_" + suffix;
  process.parameter_mappings = {{"child_pid_file", "child_pid_file"}};
  Ros2ProcessDeviceRuntimeAdapter adapter(*client_node, process, 100ms);

  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        {{"child_pid_file", pid_file.string()}}}).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(wait_until([&pid_file]() {return std::filesystem::exists(pid_file);}));
  std::ifstream pid_stream(pid_file);
  pid_t child_pid = -1;
  pid_stream >> child_pid;
  ASSERT_GT(child_pid, 0);
  const auto process_group = getpgid(child_pid);
  ASSERT_GT(process_group, 0);

  EXPECT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kFinalized, {}}).result,
    device_manager::TransitionResult::kSuccess);
  const bool group_reaped = kill(-process_group, 0) != 0 && errno == ESRCH;
  if (!group_reaped) {
    (void)kill(child_pid, SIGKILL);
  }
  std::filesystem::remove(pid_file);
  EXPECT_TRUE(group_reaped);
}

TEST_F(RosFixture, Ros2ProcessDeviceRuntimeMapsAuthoritativeParameters)
{
  auto client_node = std::make_shared<rclcpp::Node>("process_parameter_mapping_client");
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto topic = "/process_runtime_mapped_" + suffix;

  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "managed_parameter_source_" + suffix;
  process.parameter_mappings = {
    {"topic", "platform.topic"},
    {"empty_string", "platform.empty_string"},
    {"numeric_string", "platform.numeric_string"},
    {"publish_event", "platform.publish_event"},
  };
  process.topology_parameters = {{"unmapped", std::string{"topology"}}};

  Ros2ProcessDeviceRuntimeAdapter adapter(*client_node, process, 1s);
  std::atomic_bool event_received{false};
  std::atomic_bool strings_preserved{false};
  std::atomic_bool unmapped_excluded{false};
  adapter.set_event_handler(
    [&event_received, &strings_preserved, &unmapped_excluded](device_manager::DeviceEvent event) {
      if (event.code == "test.driver_fault") {
        const auto empty = event.values.find("empty_string");
        const auto numeric = event.values.find("numeric_string");
        const auto unmapped = event.values.find("unmapped");
        strings_preserved = empty != event.values.end() && empty->second.empty() &&
        numeric != event.values.end() && numeric->second == "00123";
        unmapped_excluded = unmapped != event.values.end() && unmapped->second == "topology";
        event_received = true;
      }
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);

  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        {
          {"platform.topic", topic},
          {"platform.empty_string", std::string{}},
          {"platform.numeric_string", std::string{"00123"}},
          {"platform.publish_event", true},
          {"unmapped", std::string{"must_not_be_forwarded"}},
        }}).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_TRUE(wait_until([&event_received]() {return event_received.load();}));
  EXPECT_TRUE(strings_preserved.load());
  EXPECT_TRUE(unmapped_excluded.load());
}

TEST_F(RosFixture, Ros2ProcessDeviceRuntimeRestartsWithPatchedParameters)
{
  auto client_node = std::make_shared<rclcpp::Node>("process_reconfigure_client");
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto initial_topic = "/process_runtime_initial_" + suffix;
  const auto patched_topic = "/process_runtime_patched_" + suffix;

  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "managed_reconfigure_source_" + suffix;
  process.parameter_mappings = {{"topic", "topic"}};

  Ros2ProcessDeviceRuntimeAdapter adapter(*client_node, process, 1s);
  std::atomic_bool patched_message_received{false};
  auto subscription = client_node->create_subscription<std_msgs::msg::String>(
    patched_topic, rclcpp::SensorDataQoS(),
    [&patched_message_received](const std_msgs::msg::String &) {
      patched_message_received = true;
    });
  (void)subscription;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);

  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured, {{"topic", initial_topic}}}).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive, {}}).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive, {}}).result,
    device_manager::TransitionResult::kSuccess);

  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kActive,
        device_manager::LifecycleState::kInactive, {}}).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kUnconfigured, {}}).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive, {{"topic", patched_topic}}}).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kInactive,
        device_manager::LifecycleState::kActive, {}}).result,
    device_manager::TransitionResult::kSuccess);

  EXPECT_TRUE(wait_until([&patched_message_received]() {
      return patched_message_received.load();
    }));
}

TEST_F(RosFixture, UnconfiguredManagedProcessAppliesManagerParameterPatch)
{
  auto client_node = std::make_shared<rclcpp::Node>("unconfigured_process_patch_client");
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto initial_topic = "/unconfigured_process_initial_" + suffix;
  const auto patched_topic = "/unconfigured_process_patched_" + suffix;

  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "unconfigured_process_source_" + suffix;
  process.parameter_mappings = {{"topic", "topic"}};

  std::vector<device_manager::DeviceRegistration> registrations;
  registrations.push_back({
        {"managed_process", "test", true, {{"topic", initial_topic}}},
        std::make_unique<Ros2ProcessDeviceRuntimeAdapter>(*client_node, process, 1s)});
  device_manager::DeviceManager manager(std::move(registrations));
  std::atomic_bool patched_message_received{false};
  auto subscription = client_node->create_subscription<std_msgs::msg::String>(
    patched_topic, rclcpp::SensorDataQoS(),
    [&patched_message_received](const std_msgs::msg::String &) {
      patched_message_received = true;
    });
  (void)subscription;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);
  manager.start();

  manager.tick();
  ASSERT_TRUE(wait_until([&manager]() {
      return manager.device("managed_process")->state ==
             device_manager::LifecycleState::kUnconfigured;
    }));
  ASSERT_TRUE(manager.patch_parameters(
      "managed_process", {{"topic", patched_topic}},
      device_manager::RequestPriority::kUrgent).accepted);

  EXPECT_TRUE(wait_until([&patched_message_received]() {
      return patched_message_received.load();
    }));
  manager.stop();
}

TEST_F(RosFixture, Ros2ProcessDeviceRuntimeForwardsDriverEvents)
{
  auto client_node = std::make_shared<rclcpp::Node>("process_event_client");
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());

  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "managed_event_source_" + suffix;
  process.parameter_mappings = {{"publish_event", "publish_event"}};

  Ros2ProcessDeviceRuntimeAdapter adapter(*client_node, process, 1s);
  std::atomic_bool event_received{false};
  adapter.set_event_handler(
    [&event_received](device_manager::DeviceEvent event) {
      if (event.code == "test.driver_fault" &&
      event.target_state == device_manager::LifecycleState::kUnconfigured)
      {
        event_received = true;
      }
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);

  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured, {{"publish_event", true}}}).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_TRUE(wait_until([&event_received]() {return event_received.load();}));
}

TEST_F(RosFixture, Ros2ProcessExitDoesNotChangeLifecycleOutsideTransition)
{
  auto client_node = std::make_shared<rclcpp::Node>("process_exit_client");
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());

  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "managed_exit_source_" + suffix;
  process.parameter_mappings = {{"exit_after_ms", "exit_after_ms"}};

  Ros2ProcessDeviceRuntimeAdapter adapter(*client_node, process, 1s);
  std::atomic_bool process_exited{false};
  adapter.set_event_handler(
    [&process_exited](device_manager::DeviceEvent event) {
      if (event.code == "ros2_process.process_exited") {
        process_exited = true;
      }
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);

  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        {{"exit_after_ms", std::int64_t{100}}}}).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(wait_until([&adapter, &process_exited]() {
      (void)adapter.state();
      return process_exited.load();
    }));
  EXPECT_EQ(adapter.state(), device_manager::LifecycleState::kUnconfigured);

  EXPECT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kFinalized, {}}).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(adapter.state(), device_manager::LifecycleState::kFinalized);
}

TEST_F(RosFixture, Ros2ProcessExitReapsAdoptedDescendants)
{
  auto client_node = std::make_shared<rclcpp::Node>("process_descendant_exit_client");
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto pid_file = std::filesystem::temp_directory_path() /
    ("device_manager_process_exit_descendant_" + suffix + ".pid");

  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "managed_descendant_exit_" + suffix;
  process.parameter_mappings = {{"child_pid_file", "child_pid_file"}};
  Ros2ProcessDeviceRuntimeAdapter adapter(*client_node, process, 1s);
  std::atomic_bool process_exited{false};
  adapter.set_event_handler(
    [&process_exited](device_manager::DeviceEvent event) {
      process_exited = event.code == "ros2_process.process_exited";
    });

  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured,
        {{"child_pid_file", pid_file.string()}}}).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_TRUE(wait_until([&pid_file]() {return std::filesystem::exists(pid_file);}));
  std::ifstream pid_stream(pid_file);
  pid_t descendant_pid = -1;
  pid_stream >> descendant_pid;
  ASSERT_GT(descendant_pid, 0);
  const auto process_group = getpgid(descendant_pid);
  ASSERT_GT(process_group, 0);
  ASSERT_EQ(kill(-process_group, SIGKILL), 0);

  ASSERT_TRUE(wait_until([&adapter, &process_exited]() {
      (void)adapter.state();
      return process_exited.load();
    }));
  const bool descendant_reaped = !std::filesystem::exists(
    "/proc/" + std::to_string(descendant_pid));
  if (!descendant_reaped) {
    (void)kill(descendant_pid, SIGKILL);
  }
  std::filesystem::remove(pid_file);
  EXPECT_TRUE(descendant_reaped);
}

TEST_F(RosFixture, Ros2ProcessHookConsumesReachedEventBeforeAutomaticRestart)
{
  auto client_node = std::make_shared<rclcpp::Node>("process_hook_client");
  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "managed_hook_source";

  Ros2ProcessDeviceRuntimeAdapter adapter(*client_node, process, 1s);
  device_manager::LifecycleHook hook;
  adapter.register_hook(
    [&hook](device_manager::LifecycleHook value) {hook = std::move(value);});
  ASSERT_TRUE(static_cast<bool>(hook));

  device_manager::HookContext context;
  context.enabled = true;
  context.state = device_manager::LifecycleState::kFinalized;
  context.latest_event = device_manager::DeviceEvent{
    device_manager::Clock::now(), device_manager::EventLevel::kError,
    "ros2_process.process_exited", "managed ROS process exited", "ros2_process",
    device_manager::LifecycleState::kFinalized, {}};
  context.latest_event_revision = 1;

  const auto requests = hook(context);
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().from_state, device_manager::LifecycleState::kFinalized);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kUnconfigured);

  context.state = device_manager::LifecycleState::kUnconfigured;
  const auto next_requests = hook(context);
  ASSERT_EQ(next_requests.size(), 1U);
  EXPECT_EQ(next_requests.front().target_state, device_manager::LifecycleState::kInactive);
}

TEST_F(RosFixture, DisabledRos2ProcessHookFinalizesAnActiveProcess)
{
  auto client_node = std::make_shared<rclcpp::Node>("disabled_process_hook_client");
  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "disabled_managed_process";

  Ros2ProcessDeviceRuntimeAdapter adapter(*client_node, process, 1s);
  device_manager::LifecycleHook hook;
  adapter.register_hook(
    [&hook](device_manager::LifecycleHook value) {hook = std::move(value);});
  ASSERT_TRUE(static_cast<bool>(hook));

  device_manager::HookContext context;
  context.enabled = false;
  context.state = device_manager::LifecycleState::kActive;
  const auto requests = hook(context);

  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().from_state, device_manager::LifecycleState::kActive);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kFinalized);
}

TEST_F(RosFixture, Ros2ProcessExitChangesStateOnlyThroughManagerQueue)
{
  auto client_node = std::make_shared<rclcpp::Node>("process_manager_client");
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "managed_manager_source_" + suffix;
  process.parameter_mappings = {{"exit_after_ms", "exit_after_ms"}};

  std::vector<device_manager::DeviceRegistration> registrations;
  registrations.push_back({
        {"managed_process", "test", true, {{"exit_after_ms", std::int64_t{300}}}},
        std::make_unique<Ros2ProcessDeviceRuntimeAdapter>(*client_node, process, 1s)});
  device_manager::DeviceManager manager(std::move(registrations));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);
  manager.start();

  manager.tick();
  ASSERT_TRUE(wait_until([&manager]() {
      return manager.device("managed_process")->state ==
             device_manager::LifecycleState::kUnconfigured;
    }));
  const auto materialized = manager.device("managed_process");
  ASSERT_TRUE(materialized->last_transition.has_value());
  ASSERT_EQ(
    std::get<std::int64_t>(
      materialized->last_transition->request.parameters.at("exit_after_ms")),
    300);
  manager.tick();
  ASSERT_TRUE(wait_until([&manager]() {
      return manager.device("managed_process")->state ==
             device_manager::LifecycleState::kInactive;
    }));
  manager.tick();
  ASSERT_TRUE(wait_until([&manager]() {
      return manager.device("managed_process")->state ==
             device_manager::LifecycleState::kActive;
    }));

  ASSERT_TRUE(wait_until([&manager]() {
      const auto device = manager.device("managed_process");
      return device->latest_event &&
             device->latest_event->code == "ros2_process.process_exited";
    }));
  EXPECT_EQ(
    manager.device("managed_process")->state, device_manager::LifecycleState::kActive);

  manager.tick();
  ASSERT_TRUE(wait_until([&manager]() {
      return manager.device("managed_process")->state ==
             device_manager::LifecycleState::kFinalized;
    }));
  const auto device = manager.device("managed_process");
  ASSERT_TRUE(device->last_transition.has_value());
  EXPECT_EQ(
    device->last_transition->request.target_state,
    device_manager::LifecycleState::kFinalized);
  manager.stop();
}

TEST_F(RosFixture, Ros2ProcessDeviceRuntimeSpawnsChildIntoConfiguredNamespace)
{
  auto client_node = std::make_shared<rclcpp::Node>("process_ns_client");
  const auto suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
  const auto ns = "/ns_" + suffix;
  const auto relative_topic = "scan";
  const auto resolved_topic = ns + "/" + relative_topic;

  Ros2ProcessDeviceRuntimeAdapter::ProcessSpec process;
  process.package = "device_manager_ros";
  process.executable = "device_manager_ros_test_topic_source";
  process.node_name = "managed_ns_topic_source";
  process.namespace_ = ns;
  process.parameter_mappings = {{"topic", "topic"}};

  Ros2ProcessDeviceRuntimeAdapter adapter(
    *client_node, process, 2s);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);

  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kFinalized,
        device_manager::LifecycleState::kUnconfigured, {{"topic", relative_topic}}}).result,
    device_manager::TransitionResult::kSuccess);
  ASSERT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive, {}}).result,
    device_manager::TransitionResult::kSuccess);
  const auto activated = adapter.request_transition(
    {device_manager::LifecycleState::kInactive,
      device_manager::LifecycleState::kActive, {}});
  EXPECT_EQ(activated.result, device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(adapter.state(), device_manager::LifecycleState::kActive);

  EXPECT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kActive,
        device_manager::LifecycleState::kFinalized, {}}).result,
    device_manager::TransitionResult::kSuccess);
  EXPECT_EQ(adapter.state(), device_manager::LifecycleState::kFinalized);
}

TEST_F(RosFixture, RegistrationLoaderRejectsRemovedRos2ExternalRuntime)
{
  auto client_node = std::make_shared<rclcpp::Node>("external_registration_client");
  const auto path = std::filesystem::temp_directory_path() /
    "device-manager-ros2-external.yaml";
  std::ofstream file(path);
  file <<
    "instances:\n"
    "  - device_id: front_lidar\n"
    "    device_type: lidar\n"
    "    runtime: ros2_external\n"
    "    active_topics:\n"
    "      - name: /scan\n"
    "        type: sensor_msgs/msg/LaserScan\n"
    "    topic_timeout_ms: 1000\n";
  file.close();

  EXPECT_ANY_THROW(
    load_device_registrations(
      *client_node, path.string(), 500ms,
      [](const std::string &) {
        return device_manager::ParameterMap{{"device.enable", true}};
      }));
}

TEST_F(RosFixture, RegistrationLoaderCanCreateRos2ProcessRuntime)
{
  auto client_node = std::make_shared<rclcpp::Node>("process_registration_client");
  const auto path = std::filesystem::temp_directory_path() /
    "device-manager-ros2-process.yaml";
  std::ofstream file(path);
  file <<
    "instances:\n"
    "  - device_id: front_lidar\n"
    "    device_type: lidar\n"
    "    runtime: ros2_process\n"
    "    process:\n"
    "      package: device_manager_ros\n"
    "      executable: device_manager_ros_test_topic_source\n"
    "      node_name: managed_topic_source\n"
    "      parameter_mappings:\n"
    "        topic: platform.topic\n";
  file.close();

  auto registrations = load_device_registrations(
    *client_node, path.string(), 500ms,
    [](const std::string &) {
      return device_manager::ParameterMap{
      {"device.enable", true},
      {"platform.topic", std::string{"/process_runtime_config_ready"}}};
    });
  device_manager::DeviceManager manager(std::move(registrations));

  const auto device = manager.device("front_lidar");
  ASSERT_TRUE(device.has_value());
  EXPECT_EQ(device->type, "lidar");
  EXPECT_EQ(device->state, device_manager::LifecycleState::kFinalized);
}

TEST_F(RosFixture, RegistrationLoaderKeepsTopologyParametersOutOfDeviceSnapshot)
{
  auto client_node = std::make_shared<rclcpp::Node>("topology_parameter_registration_client");
  const auto path = std::filesystem::temp_directory_path() /
    "device-manager-topology-parameters.yaml";
  std::ofstream file(path);
  file <<
    "instances:\n"
    "  - device_id: camera\n"
    "    device_type: camera\n"
    "    runtime: ros2_process\n"
    "    process:\n"
    "      package: device_manager_ros\n"
    "      executable: device_manager_ros_test_topic_source\n"
    "      topology_parameters:\n"
    "        device_num: {type: int32, value: 2}\n";
  file.close();

  auto registrations = load_device_registrations(
    *client_node, path.string(), 500ms,
    [](const std::string &) {return device_manager::ParameterMap{{"device.enable", false}};});
  device_manager::DeviceManager manager(std::move(registrations));

  const auto device = manager.device("camera");
  ASSERT_TRUE(device.has_value());
  EXPECT_EQ(device->parameters.find("device_num"), device->parameters.end());
}

TEST_F(RosFixture, RegistrationLoaderRejectsFixedProcessParameters)
{
  auto client_node = std::make_shared<rclcpp::Node>("fixed_parameter_registration_client");
  const auto path = std::filesystem::temp_directory_path() /
    "device-manager-fixed-process-parameters.yaml";
  std::ofstream file(path);
  file <<
    "instances:\n"
    "  - device_id: front_lidar\n"
    "    device_type: lidar\n"
    "    runtime: ros2_process\n"
    "    process:\n"
    "      package: device_manager_ros\n"
    "      executable: device_manager_ros_test_topic_source\n"
    "      fixed_parameters:\n"
    "        publish_event: {type: bool, value: false}\n";
  file.close();

  EXPECT_THROW(
    load_device_registrations(
      *client_node, path.string(), 500ms,
      [](const std::string &) {return device_manager::ParameterMap{};}),
    std::runtime_error);
}

TEST_F(RosFixture, RegistrationLoaderCanCreateRuntimePlugin)
{
  auto client_node = std::make_shared<rclcpp::Node>("plugin_registration_client");
  const auto path = std::filesystem::temp_directory_path() /
    "device-manager-runtime-plugin.yaml";
  std::ofstream file(path);
  file <<
    "instances:\n"
    "  - device_id: plugin_device\n"
    "    device_type: sensor\n"
    "    runtime: device_manager_ros/TestRuntimePlugin\n";
  file.close();

  auto registrations = load_device_registrations(
    *client_node, path.string(), 500ms,
    [](const std::string &) {
      return device_manager::ParameterMap{
      {"device.enable", true}, {"marker", std::string{"loaded"}}};
    });
  device_manager::DeviceManager manager(std::move(registrations));

  const auto device = manager.device("plugin_device");
  ASSERT_TRUE(device.has_value());
  EXPECT_EQ(device->type, "sensor");
  EXPECT_EQ(device->state, device_manager::LifecycleState::kUnconfigured);
}

TEST_F(RosFixture, RegistrationLoaderPassesTopologyParametersToRuntimePlugin)
{
  auto client_node = std::make_shared<rclcpp::Node>("plugin_topology_registration_client");
  const auto path = std::filesystem::temp_directory_path() /
    "device-manager-runtime-plugin-topology.yaml";
  std::ofstream file(path);
  file <<
    "instances:\n"
    "  - device_id: plugin_device\n"
    "    device_type: sensor\n"
    "    runtime: device_manager_ros/TestRuntimePlugin\n"
    "    topology_parameters:\n"
    "      topology_marker:\n"
    "        type: string\n"
    "        value: from_yaml\n";
  file.close();

  auto registrations = load_device_registrations(
    *client_node, path.string(), 500ms,
    [](const std::string &) {
      return device_manager::ParameterMap{
      {"device.enable", true}, {"marker", std::string{"loaded"}}};
    });
  device_manager::DeviceManager manager(std::move(registrations));

  const auto device = manager.device("plugin_device");
  ASSERT_TRUE(device.has_value());
  EXPECT_EQ(device->state, device_manager::LifecycleState::kInactive);
}

TEST_F(RosFixture, RegistrationLoaderUsesAuthoritativeParameterProvider)
{
  auto client_node = std::make_shared<rclcpp::Node>("provider_registration_client");
  const auto path = std::filesystem::temp_directory_path() /
    "device-manager-parameter-provider.yaml";
  std::ofstream file(path);
  file <<
    "instances:\n"
    "  - device_id: plugin_device\n"
    "    device_type: sensor\n"
    "    runtime: device_manager_ros/TestRuntimePlugin\n";
  file.close();

  auto registrations = load_device_registrations(
    *client_node, path.string(), 500ms,
    [](const std::string & id) {
      EXPECT_EQ(id, "plugin_device");
      return device_manager::ParameterMap{
      {"device.enable", false}, {"marker", std::string{"from_api"}}};
    });
  device_manager::DeviceManager manager(std::move(registrations));

  const auto device = manager.device("plugin_device");
  ASSERT_TRUE(device.has_value());
  EXPECT_FALSE(device->enabled);
}

TEST_F(RosFixture, ParameterApiResponseUsesDeclaredTypesAndSkipsCategories)
{
  const auto parameters =
    parse_device_parameters_response(
        R"json({
    "data": {
      "items": {
        "device": {"is_category": true},
        "device.enable": {"type": "bool", "value": false},
        "serial_number": {"type": "string", "value": "00123"},
        "gain": {"type": "float64", "value": 2.5}
      }
    }
  })json");

  EXPECT_EQ(parameters.size(), 3U);
  EXPECT_EQ(std::get<bool>(parameters.at("device.enable")), false);
  EXPECT_EQ(std::get<std::string>(parameters.at("serial_number")), "00123");
  EXPECT_EQ(std::get<double>(parameters.at("gain")), 2.5);
}

TEST_F(RosFixture, RegistrationLoaderRejectsYamlParameterValues)
{
  auto client_node = std::make_shared<rclcpp::Node>("yaml_values_registration_client");
  const auto path = std::filesystem::temp_directory_path() /
    "device-manager-yaml-values.yaml";
  std::ofstream file(path);
  file <<
    "instances:\n"
    "  - device_id: plugin_device\n"
    "    device_type: sensor\n"
    "    runtime: device_manager_ros/TestRuntimePlugin\n"
    "    values:\n"
    "      device.enable: true\n";
  file.close();

  EXPECT_THROW(
    load_device_registrations(
      *client_node, path.string(), 500ms,
      [](const std::string &) {return device_manager::ParameterMap{};}),
    std::runtime_error);
}

TEST_F(RosFixture, LifecycleAdapterObservesTransitionEvents)
{
  auto client_node = std::make_shared<rclcpp::Node>("transition_event_client");
  auto publisher_node = std::make_shared<rclcpp::Node>("transition_event_source");
  auto publisher = publisher_node->create_publisher<lifecycle_msgs::msg::TransitionEvent>(
    "/event_target/transition_event", rclcpp::QoS(10));
  Ros2LifecycleDriverAdapter adapter(*client_node, "/event_target", 100ms);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  executor.add_node(publisher_node);
  ExecutorThread spin_thread(executor);

  ASSERT_TRUE(wait_until([&publisher]() {return publisher->get_subscription_count() == 1U;}));
  lifecycle_msgs::msg::TransitionEvent event;
  event.goal_state.id = lifecycle_msgs::msg::State::TRANSITION_STATE_DEACTIVATING;
  publisher->publish(event);

  EXPECT_TRUE(wait_until([&adapter]() {
      return adapter.state() == device_manager::LifecycleState::kDeactivating;
  }));
}

TEST_F(RosFixture, LifecycleHookUsesCurrentEnableState)
{
  auto client_node = std::make_shared<rclcpp::Node>("disabled_adapter_client");
  Ros2LifecycleDriverAdapter adapter(*client_node, "/missing_target", 10ms);
  device_manager::LifecycleHook hook;
  adapter.register_hook([&hook](device_manager::LifecycleHook value) {
      hook = std::move(value);
  });
  ASSERT_TRUE(static_cast<bool>(hook));

  device_manager::HookContext context;
  context.state = device_manager::LifecycleState::kUnconfigured;
  context.enabled = false;
  EXPECT_TRUE(hook(context).empty());

  context.enabled = true;
  const auto requests = hook(context);
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kInactive);
}

TEST_F(RosFixture, LifecycleAdapterReportsUnavailableServiceAsError)
{
  auto client_node = std::make_shared<rclcpp::Node>("missing_adapter_client");
  Ros2LifecycleDriverAdapter adapter(*client_node, "/missing_target", 10ms);

  EXPECT_EQ(
    adapter.request_transition(
      {device_manager::LifecycleState::kUnconfigured,
        device_manager::LifecycleState::kInactive, {}}).result,
    device_manager::TransitionResult::kError);
}

TEST_F(RosFixture, LifecycleHookPrioritizesFaultTargetThenResumesAutomaticStartup)
{
  auto client_node = std::make_shared<rclcpp::Node>("hook_adapter_client");
  Ros2LifecycleDriverAdapter adapter(*client_node, "/hook_target", 10ms);
  device_manager::LifecycleHook hook;
  adapter.register_hook([&hook](device_manager::LifecycleHook value) {
      hook = std::move(value);
  });
  ASSERT_TRUE(static_cast<bool>(hook));

  device_manager::HookContext context;
  context.enabled = true;
  context.state = device_manager::LifecycleState::kUnconfigured;
  context.parameters = {{"frame_id", std::string{"camera_link"}}};
  auto requests = hook(context);
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kInactive);
  EXPECT_EQ(requests.front().parameters, context.parameters);

  context.state = device_manager::LifecycleState::kActive;
  const auto event_time = device_manager::Clock::now();
  context.latest_event = device_manager::DeviceEvent{
    event_time, device_manager::EventLevel::kError, "camera.link_lost", "link lost",
    "camera_sdk", device_manager::LifecycleState::kUnconfigured, {}};
  context.latest_event_revision = 1;
  requests = hook(context);
  ASSERT_EQ(requests.size(), 2U);
  EXPECT_EQ(requests[0].target_state, device_manager::LifecycleState::kInactive);
  EXPECT_EQ(requests[1].target_state, device_manager::LifecycleState::kUnconfigured);

  context.state = device_manager::LifecycleState::kUnconfigured;
  requests = hook(context);
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kInactive);

  context.state = device_manager::LifecycleState::kInactive;
  requests = hook(context);
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kActive);

  context.latest_event.reset();
  context.state = device_manager::LifecycleState::kInactive;
  requests = hook(context);
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kActive);
}

TEST_F(RosFixture, LifecycleHookDistinguishesEventsByRevision)
{
  auto client_node = std::make_shared<rclcpp::Node>("hook_revision_client");
  Ros2LifecycleDriverAdapter adapter(*client_node, "/hook_revision_target", 10ms);
  device_manager::LifecycleHook hook;
  adapter.register_hook([&hook](device_manager::LifecycleHook value) {
      hook = std::move(value);
  });
  ASSERT_TRUE(static_cast<bool>(hook));

  device_manager::HookContext context;
  context.enabled = true;
  context.state = device_manager::LifecycleState::kActive;
  context.latest_event = device_manager::DeviceEvent{
    {}, device_manager::EventLevel::kError, "device.degraded", "degraded",
    "driver", device_manager::LifecycleState::kInactive, {}};
  context.latest_event_revision = 1;
  auto requests = hook(context);
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kInactive);

  context.state = device_manager::LifecycleState::kInactive;
  requests = hook(context);
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kActive);

  context.latest_event = device_manager::DeviceEvent{
    {}, device_manager::EventLevel::kError, "device.reset_required", "reset required",
    "driver", device_manager::LifecycleState::kUnconfigured, {}};
  context.latest_event_revision = 2;
  requests = hook(context);
  ASSERT_EQ(requests.size(), 1U);
  EXPECT_EQ(requests.front().target_state, device_manager::LifecycleState::kUnconfigured);
}

TEST_F(RosFixture, LifecycleAdapterForwardsDeviceEvents)
{
  auto client_node = std::make_shared<rclcpp::Node>("device_event_client");
  auto publisher_node = std::make_shared<rclcpp::Node>("device_event_source");
  auto publisher = publisher_node->create_publisher<device_manager_msgs::msg::DeviceEvent>(
    "/event_device/device_event", rclcpp::QoS(10));
  Ros2LifecycleDriverAdapter adapter(*client_node, "/event_device", 100ms);
  std::atomic<bool> forwarded{false};
  adapter.set_event_handler(
    [&forwarded](device_manager::DeviceEvent event) {
      forwarded = event.level == device_manager::EventLevel::kError &&
      event.code == "camera.link_lost" && event.source == "camera_sdk";
    });
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  executor.add_node(publisher_node);
  ExecutorThread spin_thread(executor);

  ASSERT_TRUE(wait_until([&publisher]() {return publisher->get_subscription_count() == 1U;}));
  device_manager_msgs::msg::DeviceEvent event;
  event.level = device_manager_msgs::msg::DeviceEvent::ERROR;
  event.code = "camera.link_lost";
  event.message = "link lost";
  event.source = "camera_sdk";
  event.target_state = device_manager_msgs::msg::DeviceState::UNCONFIGURED;
  publisher->publish(event);

  EXPECT_TRUE(wait_until([&forwarded]() {return forwarded.load();}));
}

TEST_F(RosFixture, DeviceManagerApiDoesNotPublishDeviceEventTopic)
{
  auto manager_node = std::make_shared<rclcpp::Node>("eventless_device_manager");
  auto runtime = std::make_unique<Ros2LifecycleDriverAdapter>(
    *manager_node, "/unused_eventless_device", 20ms);
  std::vector<device_manager::DeviceRegistration> registrations;
  registrations.push_back({{"camera", "camera", true, {}}, std::move(runtime)});
  device_manager::DeviceManager manager(std::move(registrations));
  device_manager::DeviceManagerApplication application(manager, 1h);
  Ros2DeviceManagerApi api(application, *manager_node, 20ms, "/device_manager");
  application.start();
  api.start();

  EXPECT_EQ(manager_node->count_publishers("/eventless_device_manager/device_event"), 0U);
}

TEST_F(RosFixture, DeviceManagerApiUsesTheHostingNodeNamespace)
{
  auto manager_node = std::make_shared<rclcpp::Node>("embedded_manager");
  auto runtime = std::make_unique<Ros2LifecycleDriverAdapter>(
    *manager_node, "/unused_embedded_device", 20ms);
  std::vector<device_manager::DeviceRegistration> registrations;
  registrations.push_back({{"camera", "camera", true, {}}, std::move(runtime)});
  device_manager::DeviceManager manager(std::move(registrations));
  device_manager::DeviceManagerApplication application(manager, 1h);
  Ros2DeviceManagerApi api(application, *manager_node, 20ms);
  application.start();
  api.start();

  EXPECT_EQ(manager_node->count_publishers("/embedded_manager/devices"), 1U);
  EXPECT_EQ(manager_node->count_publishers("/device_manager/devices"), 0U);
  const auto services = manager_node->get_service_names_and_types();
  EXPECT_NE(services.find("/embedded_manager/get_devices"), services.end());
  EXPECT_NE(services.find("/embedded_manager/change_device_state"), services.end());
  EXPECT_NE(services.find("/embedded_manager/patch_parameters"), services.end());
}

TEST_F(RosFixture, PlatformNodeDiscoversDisabledDeviceAndServesItsSnapshot)
{
  auto manager_node = std::make_shared<rclcpp::Node>("device_manager");
  auto registrations = load_device_registrations(
    *manager_node,
    (std::filesystem::path(__FILE__).parent_path() / "device-config.yaml").string(), 1s,
    [](const std::string &) {
      return device_manager::ParameterMap{
      {"device.enable", false},
      {"frame_id", std::string{"camera_link"}},
      {"gain", 2.5},
      {"serial_number", std::string{"00123"}}};
    });
  device_manager::DeviceManager manager(std::move(registrations));
  device_manager::DeviceManagerApplication application(manager, 20ms);
  Ros2DeviceManagerApi api(application, *manager_node, 20ms);
  application.start();
  api.start();
  auto client_node = std::make_shared<rclcpp::Node>("platform_test_client");
  auto get_devices =
    client_node->create_client<device_manager_msgs::srv::GetDevices>(
    "/device_manager/get_devices");
  auto change_state =
    client_node->create_client<device_manager_msgs::srv::ChangeDeviceState>(
    "/device_manager/change_device_state");
  auto patch_parameters =
    client_node->create_client<device_manager_msgs::srv::PatchDeviceParameters>(
    "/device_manager/patch_parameters");
  std::atomic<bool> published{false};
  auto subscription = client_node->create_subscription<device_manager_msgs::msg::DeviceStateArray>(
    "/device_manager/devices", rclcpp::QoS(10),
    [&published](const device_manager_msgs::msg::DeviceStateArray & message) {
      published = message.devices.size() == 1U &&
      message.devices.front().lifecycle_state ==
      device_manager_msgs::msg::DeviceState::UNKNOWN;
    });
  (void)subscription;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(manager_node);
  executor.add_node(client_node);
  ExecutorThread spin_thread(executor);

  ASSERT_TRUE(get_devices->wait_for_service(1s));
  auto get_future = get_devices->async_send_request(
    std::make_shared<device_manager_msgs::srv::GetDevices::Request>());
  ASSERT_EQ(get_future.wait_for(1s), std::future_status::ready);
  const auto get_response = get_future.get();
  const auto & devices = get_response->result.devices;
  ASSERT_EQ(devices.size(), 1U);
  EXPECT_EQ(devices.front().id, "camera");
  EXPECT_FALSE(devices.front().enabled);
  EXPECT_EQ(
    devices.front().lifecycle_state,
    device_manager_msgs::msg::DeviceState::UNKNOWN);
  ASSERT_EQ(devices.front().parameters.size(), 4U);
  EXPECT_EQ(devices.front().parameters[0].name, "device.enable");
  EXPECT_FALSE(rclcpp::Parameter::from_parameter_msg(devices.front().parameters[0]).as_bool());
  EXPECT_EQ(devices.front().parameters[1].name, "frame_id");
  EXPECT_EQ(
    rclcpp::Parameter::from_parameter_msg(devices.front().parameters[1]).as_string(),
    "camera_link");
  EXPECT_EQ(devices.front().parameters[2].name, "gain");
  EXPECT_DOUBLE_EQ(
    rclcpp::Parameter::from_parameter_msg(devices.front().parameters[2]).as_double(), 2.5);
  EXPECT_EQ(devices.front().parameters[3].name, "serial_number");
  EXPECT_EQ(
    rclcpp::Parameter::from_parameter_msg(devices.front().parameters[3]).as_string(), "00123");

  auto change_request =
    std::make_shared<device_manager_msgs::srv::ChangeDeviceState::Request>();
  change_request->device_id = "camera";
  device_manager_msgs::msg::TransitionRequest transition;
  transition.from_state = device_manager_msgs::msg::DeviceState::FINALIZED;
  transition.target_state = device_manager_msgs::msg::DeviceState::UNCONFIGURED;
  change_request->requests.push_back(transition);
  auto change_future = change_state->async_send_request(change_request);
  ASSERT_EQ(change_future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(change_future.get()->accepted);

  auto patch_request =
    std::make_shared<device_manager_msgs::srv::PatchDeviceParameters::Request>();
  patch_request->device_id = "camera";
  patch_request->parameters.push_back(rclcpp::Parameter("gain", 2.0).to_parameter_msg());
  auto patch_future = patch_parameters->async_send_request(patch_request);
  ASSERT_EQ(patch_future.wait_for(1s), std::future_status::ready);
  EXPECT_TRUE(patch_future.get()->accepted);
  EXPECT_TRUE(wait_until([&published]() {return published.load();}));
}

}  // namespace
}  // namespace device_manager_ros
