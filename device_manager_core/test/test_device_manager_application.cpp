#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "device_manager_core/device_manager_api.hpp"
#include "device_manager_core/device_manager_application.hpp"
#include "in_process_driver.hpp"

namespace device_manager
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
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

class SharedRuntime final : public IDeviceRuntime
{
public:
  explicit SharedRuntime(std::shared_ptr<IDeviceRuntime> runtime)
  : runtime_(std::move(runtime))
  {
  }

  LifecycleState state() const override
  {
    return runtime_->state();
  }

  TransitionOutcome request_transition(const TransitionRequest & request) override
  {
    return runtime_->request_transition(request);
  }

  std::vector<TransitionRequest> parameter_transitions(
    LifecycleState state, const ParameterMap & parameters, const ParameterMap & patch) override
  {
    return runtime_->parameter_transitions(state, parameters, patch);
  }

  bool materialized() const override
  {
    return runtime_->materialized();
  }

  TransitionOutcome materialize(const ParameterMap & parameters) override
  {
    return runtime_->materialize(parameters);
  }

  TransitionOutcome dematerialize() override
  {
    return runtime_->dematerialize();
  }

  void set_event_handler(EventHandler handler) override
  {
    runtime_->set_event_handler(std::move(handler));
  }

  void register_hook(HookRegistrar registrar) override
  {
    runtime_->register_hook(std::move(registrar));
  }

private:
  std::shared_ptr<IDeviceRuntime> runtime_;
};

DeviceRegistration registration(
  std::shared_ptr<IDeviceRuntime> runtime, ParameterMap parameters = {})
{
  return {
    DeviceDefinition{"camera", "camera", true, std::move(parameters)},
    std::make_unique<SharedRuntime>(std::move(runtime))};
}

template<typename ... Registrations>
std::vector<DeviceRegistration> registration_list(Registrations && ... registrations)
{
  std::vector<DeviceRegistration> result;
  result.reserve(sizeof...(Registrations));
  (result.push_back(std::forward<Registrations>(registrations)), ...);
  return result;
}

TransitionRequest transition(LifecycleState from, LifecycleState target)
{
  return {from, target, {}};
}

class TestApi final : public DeviceManagerApi
{
public:
  using DeviceManagerApi::DeviceManagerApi;

  void start() override {}
  void stop() override {}

  IDeviceManagerApplication & exposed_application()
  {
    return application();
  }
};

static_assert(!std::is_copy_constructible_v<DeviceManagerApi>);
static_assert(!std::is_copy_assignable_v<DeviceManagerApi>);

TEST(DeviceManagerApplication, DrivesLifecycleHooksOnPeriodicTicks)
{
  auto driver = std::make_shared<InProcessDeviceDriver>(
    LifecycleState::kUnconfigured, InProcessDeviceDriver::TransitionHandler{},
    [](const HookContext & context) {
      if (context.state == LifecycleState::kUnconfigured) {
        return std::vector<TransitionRequest>{
        transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)};
      }
      if (context.state == LifecycleState::kInactive) {
        return std::vector<TransitionRequest>{
        transition(LifecycleState::kInactive, LifecycleState::kActive)};
      }
      return std::vector<TransitionRequest>{};
    });
  DeviceManager manager(registration_list(registration(driver)));
  DeviceManagerApplication application(manager, 5ms);
  IDeviceManagerApplication & port = application;

  application.start();

  ASSERT_TRUE(wait_until([&port]() {
      const auto device = port.device("camera");
      return device && device->state == LifecycleState::kActive;
    }));
  application.stop();
}

TEST(DeviceManagerApplication, WaitsOnePeriodBeforeTheFirstTick)
{
  std::atomic<int> hook_calls{0};
  auto driver = std::make_shared<InProcessDeviceDriver>(
    LifecycleState::kUnconfigured, InProcessDeviceDriver::TransitionHandler{},
    [&hook_calls](const HookContext &) {
      ++hook_calls;
      return std::vector<TransitionRequest>{};
    });
  DeviceManager manager(registration_list(registration(driver)));
  DeviceManagerApplication application(manager, 1s);

  application.start();

  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(hook_calls, 0);
  application.stop();
}

TEST(DeviceManagerApplication, DelegatesStateChangesAndQueriesThroughPort)
{
  auto driver = std::make_shared<InProcessDeviceDriver>(LifecycleState::kUnconfigured);
  DeviceManager manager(registration_list(registration(driver)));
  DeviceManagerApplication application(manager, 1h);
  IDeviceManagerApplication & port = application;
  application.start();

  ASSERT_TRUE(port.change_state(
    "camera", {transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(wait_until([&port]() {
      const auto device = port.device("camera");
      return device && device->state == LifecycleState::kInactive;
    }));

  EXPECT_EQ(port.devices().size(), 1U);
  application.stop();
}

TEST(DeviceManagerApplication, DelegatesParameterPatchesThroughPort)
{
  auto driver = std::make_shared<InProcessDeviceDriver>(LifecycleState::kInactive);
  DeviceManager manager(registration_list(registration(driver, {{"gain", 1.0}})));
  DeviceManagerApplication application(manager, 1h);
  IDeviceManagerApplication & port = application;
  application.start();

  ASSERT_TRUE(port.patch_parameters(
    "camera", {{"gain", 2.0}}, RequestPriority::kNormal).accepted);
  const auto device = port.device("camera");
  ASSERT_TRUE(device.has_value());
  EXPECT_EQ(device->parameters.at("gain"), ParameterValue{2.0});
  EXPECT_FALSE(device->last_transition.has_value());
  application.stop();
}

TEST(DeviceManagerApi, ExposesTheInjectedApplicationToDerivedApis)
{
  DeviceManager manager({});
  DeviceManagerApplication application(manager, 5ms);
  TestApi api(application);

  EXPECT_EQ(&api.exposed_application(), &application);
}

TEST(DeviceManagerApplication, RejectsNonPositiveTickPeriods)
{
  DeviceManager manager({});

  EXPECT_THROW(DeviceManagerApplication(manager, 0ms), std::invalid_argument);
  EXPECT_THROW(DeviceManagerApplication(manager, -1ms), std::invalid_argument);
}

TEST(DeviceManagerApplication, StopsTheManagerOnDestruction)
{
  auto driver = std::make_shared<InProcessDeviceDriver>(LifecycleState::kUnconfigured);
  DeviceManager manager(registration_list(registration(driver)));
  {
    DeviceManagerApplication application(manager, 1h);
    application.start();
  }

  EXPECT_FALSE(manager.enqueue(
    "camera", {transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
}

TEST(DeviceManagerApplication, StopFinalizesActiveRuntimeThroughTheDeviceQueue)
{
  const auto caller_thread = std::this_thread::get_id();
  std::thread::id transition_thread;
  std::atomic<int> transition_count{0};
  auto driver = std::make_shared<InProcessDeviceDriver>(
    LifecycleState::kActive,
    [&transition_thread, &transition_count](const TransitionRequest &) {
      transition_thread = std::this_thread::get_id();
      ++transition_count;
      return TransitionOutcome{TransitionResult::kSuccess, {}};
    });
  DeviceManager manager(registration_list(registration(driver)));
  DeviceManagerApplication application(manager, 1h);
  application.start();

  application.stop();
  application.stop();

  EXPECT_EQ(driver->state(), LifecycleState::kFinalized);
  EXPECT_NE(transition_thread, caller_thread);
  EXPECT_EQ(transition_count, 1);
}

TEST(DeviceManagerApplication, RejectsWorkWhileFinalizationIsInProgress)
{
  std::mutex transition_mutex;
  std::condition_variable transition_condition;
  bool finalization_entered = false;
  bool release_finalization = false;
  auto driver = std::make_shared<InProcessDeviceDriver>(
    LifecycleState::kActive,
    [&](const TransitionRequest & request) {
      if (request.target_state != LifecycleState::kFinalized) {
        return TransitionOutcome{TransitionResult::kSuccess, {}};
      }
      std::unique_lock<std::mutex> lock(transition_mutex);
      finalization_entered = true;
      transition_condition.notify_all();
      transition_condition.wait(lock, [&release_finalization]() {return release_finalization;});
      return TransitionOutcome{TransitionResult::kSuccess, {}};
    });
  DeviceManager manager(registration_list(registration(driver)));
  DeviceManagerApplication application(manager, 1h);
  IDeviceManagerApplication & port = application;
  application.start();

  std::thread stopping([&application]() {application.stop();});
  {
    std::unique_lock<std::mutex> lock(transition_mutex);
    ASSERT_TRUE(transition_condition.wait_for(
      lock, 2s, [&finalization_entered]() {return finalization_entered;}));
  }
  EXPECT_FALSE(port.change_state(
    "camera", {transition(LifecycleState::kActive, LifecycleState::kInactive)},
    RequestPriority::kUrgent).accepted);
  {
    std::lock_guard<std::mutex> lock(transition_mutex);
    release_finalization = true;
  }
  transition_condition.notify_all();
  stopping.join();
}

TEST(DeviceManagerApplication, SerializesOverlappingStopAndStart)
{
  std::mutex hook_mutex;
  std::condition_variable hook_condition;
  bool hook_entered = false;
  bool release_hook = false;
  auto driver = std::make_shared<InProcessDeviceDriver>(
    LifecycleState::kUnconfigured, InProcessDeviceDriver::TransitionHandler{},
    [&](const HookContext &) {
      std::unique_lock<std::mutex> lock(hook_mutex);
      hook_entered = true;
      hook_condition.notify_all();
      hook_condition.wait(lock, [&release_hook]() {return release_hook;});
      return std::vector<TransitionRequest>{};
    });
  DeviceManager manager(registration_list(registration(driver)));
  DeviceManagerApplication application(manager, 1ms);
  IDeviceManagerApplication & port = application;
  application.start();
  {
    std::unique_lock<std::mutex> lock(hook_mutex);
    ASSERT_TRUE(hook_condition.wait_for(lock, 2s, [&hook_entered]() {return hook_entered;}));
  }

  std::thread stopping([&application]() {application.stop();});
  std::this_thread::sleep_for(20ms);
  std::atomic<bool> restarted{false};
  std::thread starting([&application, &restarted]() {
      application.start();
      restarted = true;
    });
  std::this_thread::sleep_for(20ms);
  {
    std::lock_guard<std::mutex> lock(hook_mutex);
    release_hook = true;
  }
  hook_condition.notify_all();

  stopping.join();
  starting.join();
  EXPECT_TRUE(restarted);
  EXPECT_TRUE(port.change_state(
    "camera", {transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  application.stop();
}

}  // namespace
}  // namespace device_manager
