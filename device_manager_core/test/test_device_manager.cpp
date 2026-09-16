#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "device_manager_core/device_manager.hpp"

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

class FakeDriver
  : public DeviceRuntimeBase
{
public:
  struct Step
  {
    TransitionResult result{TransitionResult::kSuccess};
    LifecycleState actual_state{LifecycleState::kUnknown};
    std::string message;
  };

  explicit FakeDriver(
    LifecycleState initial_state, LifecycleHook hook = {})
  : state_(initial_state), hook_(std::move(hook))
  {
  }

  LifecycleState state() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  std::vector<TransitionRequest> parameter_transitions(
    LifecycleState state, const ParameterMap & parameters, const ParameterMap & patch) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      parameter_patch_ = patch;
    }
    return standard_parameter_transitions(state, parameters);
  }

  TransitionOutcome request_transition(const TransitionRequest & request) override
  {
    const auto in_flight = ++in_flight_;
    auto current_max = max_in_flight_.load();
    while (in_flight > current_max &&
      !max_in_flight_.compare_exchange_weak(current_max, in_flight))
    {
    }

    Step step;
    bool block = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      requests_.push_back(request);
      block = block_next_;
      block_next_ = false;
      entered_ = true;
      condition_.notify_all();
      if (block) {
        condition_.wait(lock, [this]() {return released_;});
      }
      if (steps_.empty()) {
        step.actual_state = request.target_state;
      } else {
        step = std::move(steps_.front());
        steps_.pop_front();
      }
      state_ = step.actual_state;
    }

    --in_flight_;
    condition_.notify_all();
    return TransitionOutcome{step.result, std::move(step.message)};
  }

  void set_event_handler(EventHandler handler) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    event_handler_ = std::move(handler);
  }

  void register_hook(HookRegistrar registrar) override
  {
    if (hook_) {
      registrar(hook_);
    }
  }

  void emit(DeviceEvent event)
  {
    EventHandler handler;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      handler = event_handler_;
    }
    ASSERT_TRUE(static_cast<bool>(handler));
    handler(std::move(event));
  }

  void script(Step step)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    steps_.push_back(std::move(step));
  }

  void block_next_request()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    block_next_ = true;
    released_ = false;
    entered_ = false;
  }

  bool wait_until_entered()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_.wait_for(lock, 2s, [this]() {return entered_;});
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

  std::vector<TransitionRequest> requests() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return requests_;
  }

  std::size_t request_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return requests_.size();
  }

  ParameterMap parameter_patch() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return parameter_patch_;
  }

  int max_in_flight() const
  {
    return max_in_flight_.load();
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  LifecycleState state_;
  LifecycleHook hook_;
  EventHandler event_handler_;
  std::deque<Step> steps_;
  std::vector<TransitionRequest> requests_;
  ParameterMap parameter_patch_;
  bool block_next_{false};
  bool released_{false};
  bool entered_{false};
  std::atomic<int> in_flight_{0};
  std::atomic<int> max_in_flight_{0};
};

class PatchRecordingDriver final : public IDeviceRuntime
{
public:
  LifecycleState state() const override
  {
    return LifecycleState::kActive;
  }

  TransitionOutcome request_transition(const TransitionRequest &) override
  {
    ++request_count_;
    return {TransitionResult::kSuccess, {}};
  }

  std::vector<TransitionRequest> parameter_transitions(
    LifecycleState state, const ParameterMap & parameters, const ParameterMap & patch) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = state;
    parameters_ = parameters;
    patch_ = patch;
    patch_received_ = true;
    return {};
  }

  void set_event_handler(EventHandler) override {}
  void register_hook(HookRegistrar) override {}

  bool patch_received() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return patch_received_;
  }

  LifecycleState patch_state() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
  }

  ParameterMap parameters() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return parameters_;
  }

  ParameterMap patch() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return patch_;
  }

  std::size_t request_count() const
  {
    return request_count_.load();
  }

private:
  mutable std::mutex mutex_;
  LifecycleState state_{LifecycleState::kUnknown};
  ParameterMap parameters_;
  ParameterMap patch_;
  bool patch_received_{false};
  std::atomic<std::size_t> request_count_{0};
};

class ThrowingStateDriver final : public IDeviceRuntime
{
public:
  LifecycleState state() const override
  {
    throw std::runtime_error("state unavailable");
  }

  TransitionOutcome request_transition(const TransitionRequest &) override
  {
    return {TransitionResult::kSuccess, {}};
  }

  std::vector<TransitionRequest> parameter_transitions(
    LifecycleState, const ParameterMap &, const ParameterMap &) override
  {
    return {};
  }

  void set_event_handler(EventHandler) override {}
  void register_hook(HookRegistrar) override {}
};

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

  std::vector<TransitionRequest> parameter_transitions(
    LifecycleState state, const ParameterMap & parameters, const ParameterMap & patch) override
  {
    return runtime_->parameter_transitions(state, parameters, patch);
  }

private:
  std::shared_ptr<IDeviceRuntime> runtime_;
};

DeviceRegistration registration(
  std::shared_ptr<IDeviceRuntime> runtime, bool enabled = true,
  ParameterMap parameters = {})
{
  return DeviceRegistration{
    DeviceDefinition{"camera", "camera", enabled, std::move(parameters)},
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
  return TransitionRequest{from, target, {}};
}

TEST(DeviceManager, ExecutesOneTransitionAtATimePerDevice)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kUnconfigured);
  driver->block_next_request();
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(driver->wait_until_entered());
  EXPECT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kInactive, LifecycleState::kActive)},
    RequestPriority::kNormal).accepted);

  EXPECT_EQ(driver->request_count(), 1U);
  driver->release();
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 2U;}));
  EXPECT_EQ(driver->max_in_flight(), 1);
  manager.stop();
}

TEST(DeviceManager, UrgentSubmissionsPrecedeQueuedNormalSubmissions)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kUnconfigured);
  driver->block_next_request();
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(driver->wait_until_entered());
  EXPECT_TRUE(manager.enqueue(
    "camera",
      {
        transition(LifecycleState::kInactive, LifecycleState::kActive),
        transition(LifecycleState::kActive, LifecycleState::kInactive),
    }, RequestPriority::kNormal).accepted);
  EXPECT_TRUE(manager.enqueue(
    "camera",
      {
        transition(LifecycleState::kInactive, LifecycleState::kUnconfigured),
        transition(LifecycleState::kUnconfigured, LifecycleState::kInactive),
    }, RequestPriority::kUrgent).accepted);

  driver->release();
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 5U;}));
  const auto requests = driver->requests();
  EXPECT_EQ(requests[1].target_state, LifecycleState::kUnconfigured);
  EXPECT_EQ(requests[2].target_state, LifecycleState::kInactive);
  EXPECT_EQ(requests[3].target_state, LifecycleState::kActive);
  EXPECT_EQ(requests[4].target_state, LifecycleState::kInactive);
  manager.stop();
}

TEST(DeviceManager, KeepsEveryBatchAdjacentInTheQueue)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kUnconfigured);
  driver->block_next_request();
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(driver->wait_until_entered());
  EXPECT_TRUE(manager.enqueue(
    "camera",
      {
        transition(LifecycleState::kInactive, LifecycleState::kActive),
        transition(LifecycleState::kActive, LifecycleState::kInactive),
    }, RequestPriority::kNormal).accepted);
  EXPECT_TRUE(manager.enqueue(
    "camera",
      {
        transition(LifecycleState::kInactive, LifecycleState::kUnconfigured),
        transition(LifecycleState::kUnconfigured, LifecycleState::kInactive),
    }, RequestPriority::kNormal).accepted);

  driver->release();
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 5U;}));
  const auto requests = driver->requests();
  EXPECT_EQ(requests[1].target_state, LifecycleState::kActive);
  EXPECT_EQ(requests[2].target_state, LifecycleState::kInactive);
  EXPECT_EQ(requests[3].target_state, LifecycleState::kUnconfigured);
  EXPECT_EQ(requests[4].target_state, LifecycleState::kInactive);
  manager.stop();
}

TEST(DeviceManager, StopsABatchAfterTheFirstFailedTransition)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kActive);
  driver->script({TransitionResult::kSuccess, LifecycleState::kInactive, "deactivated"});
  driver->script({TransitionResult::kFailure, LifecycleState::kFinalized, "cleanup failed"});
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera",
      {
        transition(LifecycleState::kActive, LifecycleState::kInactive),
        transition(LifecycleState::kInactive, LifecycleState::kUnconfigured),
        transition(LifecycleState::kUnconfigured, LifecycleState::kInactive),
        transition(LifecycleState::kInactive, LifecycleState::kActive),
    }, RequestPriority::kNormal).accepted);
  ASSERT_TRUE(wait_until([&manager]() {
      const auto device = manager.device("camera");
      return device && device->last_transition &&
             device->last_transition->actual_state == LifecycleState::kFinalized;
    }));

  const auto record = manager.device("camera")->last_transition.value();
  EXPECT_FALSE(record.discarded);
  EXPECT_EQ(record.result, TransitionResult::kFailure);
  EXPECT_EQ(driver->request_count(), 2U);
  manager.stop();
}

TEST(DeviceManager, DiscardsARequestWhoseExpectedSourceIsStale)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kActive);
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kInactive, LifecycleState::kUnconfigured)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(wait_until([&manager]() {
      const auto device = manager.device("camera");
      return device && device->last_transition && device->last_transition->discarded;
    }));

  const auto device = manager.device("camera");
  ASSERT_TRUE(device.has_value());
  ASSERT_TRUE(device->last_transition.has_value());
  EXPECT_TRUE(device->last_transition->discarded);
  EXPECT_EQ(device->last_transition->actual_state, LifecycleState::kActive);
  EXPECT_EQ(driver->request_count(), 0U);
  manager.stop();
}

TEST(DeviceManager, ReadsActualDriverStateAfterEveryTransitionOutcome)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kUnconfigured);
  driver->script({TransitionResult::kSuccess, LifecycleState::kInactive, "configured"});
  driver->script({TransitionResult::kFailure, LifecycleState::kUnconfigured, "rejected"});
  driver->script({TransitionResult::kError, LifecycleState::kFinalized, "driver error"});
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 1U;}));
  ASSERT_TRUE(wait_until([&manager]() {
      const auto device = manager.device("camera");
      return device && device->last_transition &&
             device->last_transition->result == TransitionResult::kSuccess;
    }));
  EXPECT_EQ(manager.device("camera")->state, LifecycleState::kInactive);
  EXPECT_EQ(
    manager.device("camera")->last_transition->actual_state, LifecycleState::kInactive);

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kInactive, LifecycleState::kActive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 2U;}));
  ASSERT_TRUE(wait_until([&manager]() {
      const auto device = manager.device("camera");
      return device && device->last_transition &&
             device->last_transition->result == TransitionResult::kFailure;
    }));
  EXPECT_EQ(manager.device("camera")->state, LifecycleState::kUnconfigured);
  EXPECT_EQ(
    manager.device("camera")->last_transition->actual_state,
    LifecycleState::kUnconfigured);

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 3U;}));
  ASSERT_TRUE(wait_until([&manager]() {
      const auto device = manager.device("camera");
      return device && device->last_transition &&
             device->last_transition->result == TransitionResult::kError;
    }));
  EXPECT_EQ(manager.device("camera")->state, LifecycleState::kFinalized);
  EXPECT_EQ(
    manager.device("camera")->last_transition->actual_state, LifecycleState::kFinalized);
  manager.stop();
}

TEST(DeviceManager, RejectsReportedSuccessWhenTheTargetStateWasNotReached)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kUnconfigured);
  driver->script(
    {TransitionResult::kSuccess, LifecycleState::kUnconfigured, "reported success"});
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(wait_until([&manager]() {
      const auto device = manager.device("camera");
      return device && device->last_transition.has_value();
    }));

  EXPECT_EQ(
    manager.device("camera")->last_transition->result, TransitionResult::kFailure);
  manager.stop();
}

TEST(DeviceManager, EventUpdatesObservationWithoutChangingLifecycle)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kActive);
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();
  const DeviceEvent event{
    Clock::now(), EventLevel::kError, "camera.link_lost", "link lost", "camera_sdk",
    LifecycleState::kUnconfigured, {{"serial", "A001"}}};

  driver->emit(event);
  ASSERT_TRUE(wait_until([&manager]() {
      const auto device = manager.device("camera");
      return device && device->latest_event.has_value();
    }));

  const auto device = manager.device("camera");
  ASSERT_TRUE(device.has_value());
  ASSERT_TRUE(device->latest_event.has_value());
  EXPECT_EQ(device->latest_event->code, "camera.link_lost");
  EXPECT_EQ(device->state, LifecycleState::kActive);
  EXPECT_EQ(driver->request_count(), 0U);
  manager.stop();
}

TEST(DeviceManager, HookContextEventRevisionAdvancesForEveryDriverEvent)
{
  std::vector<std::uint64_t> revisions;
  auto driver = std::make_shared<FakeDriver>(
    LifecycleState::kActive,
    [&revisions](const HookContext & context) {
      revisions.push_back(context.latest_event_revision);
      return std::vector<TransitionRequest>{};
    });
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();

  driver->emit(DeviceEvent{});
  manager.tick();
  driver->emit(DeviceEvent{});
  manager.tick();

  ASSERT_EQ(revisions.size(), 2U);
  EXPECT_EQ(revisions[0], 1U);
  EXPECT_EQ(revisions[1], 2U);
  manager.stop();
}

TEST(DeviceManager, EveryTickCanSubmitTheDriversHookRequests)
{
  std::atomic<int> hook_calls{0};
  auto driver = std::make_shared<FakeDriver>(
    LifecycleState::kUnconfigured,
    [&hook_calls](const HookContext & context) {
      ++hook_calls;
      if (context.state == LifecycleState::kUnconfigured) {
        return std::vector<TransitionRequest>{
        transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)};
      }
      return std::vector<TransitionRequest>{
      transition(LifecycleState::kInactive, LifecycleState::kActive)};
    });
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();

  manager.tick();
  ASSERT_TRUE(wait_until([&driver]() {
      return driver->request_count() == 1U &&
             driver->state() == LifecycleState::kInactive;
    }));
  manager.tick();
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 2U;}));

  EXPECT_EQ(hook_calls.load(), 2);
  EXPECT_EQ(driver->state(), LifecycleState::kActive);
  manager.stop();
}

TEST(DeviceManager, FailedHookTransitionCanBeRetriedOnTheNextTick)
{
  auto driver = std::make_shared<FakeDriver>(
    LifecycleState::kUnconfigured,
    [](const HookContext & context) {
      return std::vector<TransitionRequest>{
      transition(context.state, LifecycleState::kInactive)};
    });
  driver->script(
    {TransitionResult::kFailure, LifecycleState::kUnconfigured, "not ready"});
  driver->script(
    {TransitionResult::kSuccess, LifecycleState::kInactive, "configured"});
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();

  manager.tick();
  ASSERT_TRUE(wait_until([&manager]() {
      const auto device = manager.device("camera");
      return device && device->last_transition &&
             device->last_transition->result == TransitionResult::kFailure;
    }));
  manager.tick();
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 2U;}));

  EXPECT_EQ(driver->state(), LifecycleState::kInactive);
  manager.stop();
}

TEST(DeviceManager, AThrowingHookDoesNotBlockOtherDevicesOrLaterTicks)
{
  std::atomic<int> healthy_hook_calls{0};
  auto throwing_driver = std::make_shared<FakeDriver>(
    LifecycleState::kActive,
    [](const HookContext &) -> std::vector<TransitionRequest> {
      throw std::runtime_error("hook failed");
    });
  auto healthy_driver = std::make_shared<FakeDriver>(
    LifecycleState::kActive,
    [&healthy_hook_calls](const HookContext &) {
      ++healthy_hook_calls;
      return std::vector<TransitionRequest>{};
    });
  DeviceManager manager(registration_list(
        registration(throwing_driver),
        DeviceRegistration{
        DeviceDefinition{"lidar", "lidar", true, {}},
        std::make_unique<SharedRuntime>(healthy_driver)}));
  manager.start();

  testing::internal::CaptureStderr();
  EXPECT_NO_THROW(manager.tick());
  EXPECT_NO_THROW(manager.tick());
  const auto error_log = testing::internal::GetCapturedStderr();

  EXPECT_NE(error_log.find("camera"), std::string::npos);
  EXPECT_NE(error_log.find("hook failed"), std::string::npos);
  EXPECT_EQ(healthy_hook_calls.load(), 2);
  manager.stop();
}

TEST(DeviceManager, CanRestartWithoutRegisteringTheHookTwice)
{
  auto driver = std::make_shared<FakeDriver>(
    LifecycleState::kUnconfigured,
    [](const HookContext & context) {
      if (context.state == LifecycleState::kFinalized) {
        return std::vector<TransitionRequest>{
        transition(LifecycleState::kFinalized, LifecycleState::kUnconfigured)};
      }
      return std::vector<TransitionRequest>{
      transition(context.state, LifecycleState::kInactive)};
    });
  DeviceManager manager(registration_list(registration(driver)));

  manager.start();
  manager.stop();
  manager.start();
  manager.tick();
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 2U;}));
  manager.tick();
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 3U;}));

  EXPECT_EQ(driver->state(), LifecycleState::kInactive);
  const auto requests = driver->requests();
  EXPECT_EQ(requests[0].target_state, LifecycleState::kFinalized);
  EXPECT_EQ(requests[1].target_state, LifecycleState::kUnconfigured);
  EXPECT_EQ(requests[2].target_state, LifecycleState::kInactive);
  manager.stop();
}

TEST(DeviceManager, ActiveParameterPatchCarriesTheCompleteMergedParameterSet)
{
  ParameterMap parameters{
    {"baud_rate", std::int64_t{9600}}, {"frame_id", std::string{"camera_link"}}};
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kActive);
  DeviceManager manager(registration_list(registration(driver, true, parameters)));
  manager.start();

  const auto result = manager.patch_parameters(
    "camera", {{"baud_rate", std::int64_t{115200}}}, RequestPriority::kUrgent);
  ASSERT_TRUE(result.accepted);
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 4U;}));

  const auto requests = driver->requests();
  ASSERT_EQ(requests.size(), 4U);
  EXPECT_EQ(requests[0].from_state, LifecycleState::kActive);
  EXPECT_EQ(requests[0].target_state, LifecycleState::kInactive);
  EXPECT_EQ(requests[1].target_state, LifecycleState::kUnconfigured);
  EXPECT_EQ(requests[2].from_state, LifecycleState::kUnconfigured);
  EXPECT_EQ(requests[2].target_state, LifecycleState::kInactive);
  EXPECT_EQ(requests[2].parameters, (ParameterMap{
        {"baud_rate", std::int64_t{115200}}, {"frame_id", std::string{"camera_link"}}}));
  EXPECT_EQ(requests[3].target_state, LifecycleState::kActive);

  const auto device = manager.device("camera");
  ASSERT_TRUE(device.has_value());
  EXPECT_EQ(device->parameters, requests[2].parameters);
  manager.stop();
}

TEST(DeviceManager, DisablingDeviceSupersedesQueuedParameterReconfiguration)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kActive);
  driver->block_next_request();
  DeviceManager manager(registration_list(registration(driver, true, {{"gain", 0.0}})));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kActive, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(driver->wait_until_entered());
  ASSERT_TRUE(manager.patch_parameters(
    "camera", {{"gain", 1.0}}, RequestPriority::kNormal).accepted);
  ASSERT_TRUE(manager.patch_parameters(
    "camera", {{"device.enable", false}, {"gain", 2.0}}, RequestPriority::kUrgent).accepted);

  driver->release();
  ASSERT_TRUE(wait_until([&manager]() {
      const auto device = manager.device("camera");
      return device && device->last_transition && device->last_transition->discarded &&
             device->last_transition->message == "superseded parameter patch";
    }));

  EXPECT_EQ(driver->request_count(), 1U);
  const auto device = manager.device("camera");
  ASSERT_TRUE(device.has_value());
  EXPECT_FALSE(device->enabled);
  EXPECT_EQ(device->parameters.at("gain"), ParameterValue{2.0});
  manager.stop();
}

TEST(DeviceManager, RuntimeReceivesTheCompleteParametersAndSubmittedPatch)
{
  auto runtime = std::make_unique<PatchRecordingDriver>();
  const auto driver = runtime.get();
  DeviceManager manager(registration_list(DeviceRegistration{
        DeviceDefinition{
          "camera", "camera", true,
          {{"gain", 1.0}, {"frame_id", std::string{"camera_link"}}}},
        std::move(runtime)}));
  manager.start();

  const ParameterMap patch{{"device.enable", true}, {"gain", 2.0}};
  ASSERT_TRUE(manager.patch_parameters("camera", patch, RequestPriority::kNormal).accepted);
  ASSERT_TRUE(wait_until([driver]() {return driver->patch_received();}));

  EXPECT_EQ(driver->patch_state(), LifecycleState::kActive);
  EXPECT_EQ(driver->patch(), patch);
  EXPECT_EQ(
    driver->parameters(),
    (ParameterMap{
        {"device.enable", true}, {"gain", 2.0},
        {"frame_id", std::string{"camera_link"}}}));
  EXPECT_EQ(driver->request_count(), 0U);
  manager.stop();
}

TEST(DeviceManager, DisabledDeviceStoresParameterPatchWithoutLifecycleChanges)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kFinalized);
  DeviceManager manager(registration_list(registration(driver, false)));
  manager.start();

  ASSERT_TRUE(manager.patch_parameters(
    "camera", {{"camera.fps", std::int64_t{30}}}, RequestPriority::kNormal).accepted);
  const auto device = manager.device("camera");
  ASSERT_TRUE(device.has_value());
  EXPECT_EQ(device->parameters.at("camera.fps"), ParameterValue{std::int64_t{30}});
  EXPECT_FALSE(wait_until([&driver]() {return driver->request_count() != 0U;}, 50ms));
  EXPECT_EQ(driver->state(), LifecycleState::kFinalized);
  manager.stop();
}

TEST(DeviceManager, ParameterPatchDerivesItsLifecyclePathWhenTheWorkerExecutesIt)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kActive);
  driver->block_next_request();
  DeviceManager manager(registration_list(registration(driver, true, {{"gain", 0.0}})));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kActive, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(driver->wait_until_entered());
  ASSERT_TRUE(manager.patch_parameters(
    "camera", {{"gain", 1.0}}, RequestPriority::kNormal).accepted);

  driver->release();
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 3U;}));
  const auto requests = driver->requests();
  EXPECT_EQ(requests[1].from_state, LifecycleState::kInactive);
  EXPECT_EQ(requests[1].target_state, LifecycleState::kUnconfigured);
  EXPECT_EQ(requests[2].from_state, LifecycleState::kUnconfigured);
  EXPECT_EQ(requests[2].target_state, LifecycleState::kInactive);
  EXPECT_EQ(requests[2].parameters.at("gain"), ParameterValue{1.0});
  manager.stop();
}

TEST(DeviceManager, SupersededParameterPatchIsNotAppliedAfterANewerUrgentPatch)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kActive);
  driver->block_next_request();
  DeviceManager manager(registration_list(registration(driver, true, {{"gain", 0.0}})));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera",
      {
        transition(LifecycleState::kActive, LifecycleState::kInactive),
        transition(LifecycleState::kInactive, LifecycleState::kActive),
    }, RequestPriority::kNormal).accepted);
  ASSERT_TRUE(driver->wait_until_entered());
  ASSERT_TRUE(manager.patch_parameters(
    "camera", {{"gain", 1.0}}, RequestPriority::kNormal).accepted);
  ASSERT_TRUE(manager.patch_parameters(
    "camera", {{"gain", 2.0}}, RequestPriority::kUrgent).accepted);

  driver->release();
  ASSERT_TRUE(wait_until([&manager, &driver]() {
      const auto device = manager.device("camera");
      return (device && device->last_transition &&
             device->last_transition->message == "superseded parameter patch") ||
             driver->request_count() == 10U;
    }));

  EXPECT_EQ(driver->request_count(), 6U);
  const auto requests = driver->requests();
  ASSERT_EQ(requests.size(), 6U);
  EXPECT_EQ(requests[4].parameters.at("gain"), ParameterValue{2.0});
  EXPECT_TRUE(manager.device("camera")->last_transition->discarded);
  manager.stop();
}

TEST(DeviceManager, NewestParameterRevisionReceivesTheAccumulatedPendingPatch)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kActive);
  driver->block_next_request();
  DeviceManager manager(registration_list(registration(
        driver, true,
      {{"gain", 0.0}, {"frame_id", std::string{"old_link"}}})));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kActive, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(driver->wait_until_entered());
  ASSERT_TRUE(manager.patch_parameters(
    "camera", {{"gain", 1.0}}, RequestPriority::kNormal).accepted);
  ASSERT_TRUE(manager.patch_parameters(
    "camera", {{"frame_id", std::string{"new_link"}}},
    RequestPriority::kUrgent).accepted);

  driver->release();
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 3U;}));

  EXPECT_EQ(
    driver->parameter_patch(),
    (ParameterMap{{"gain", 1.0}, {"frame_id", std::string{"new_link"}}}));
  manager.stop();
}

TEST(DeviceManager, AnExecutingPatchCompletesBeforeANewerPatchStarts)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kActive);
  driver->block_next_request();
  DeviceManager manager(registration_list(registration(driver, true, {{"gain", 0.0}})));
  manager.start();

  ASSERT_TRUE(manager.patch_parameters(
    "camera", {{"gain", 1.0}}, RequestPriority::kNormal).accepted);
  ASSERT_TRUE(driver->wait_until_entered());
  ASSERT_TRUE(manager.patch_parameters(
    "camera", {{"gain", 2.0}}, RequestPriority::kUrgent).accepted);

  driver->release();
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 8U;}));
  const auto requests = driver->requests();
  ASSERT_EQ(requests.size(), 8U);
  EXPECT_EQ(requests[0].target_state, LifecycleState::kInactive);
  EXPECT_EQ(requests[1].target_state, LifecycleState::kUnconfigured);
  EXPECT_EQ(requests[2].parameters.at("gain"), ParameterValue{1.0});
  EXPECT_EQ(requests[3].target_state, LifecycleState::kActive);
  EXPECT_EQ(requests[4].from_state, LifecycleState::kActive);
  EXPECT_EQ(requests[6].parameters.at("gain"), ParameterValue{2.0});
  EXPECT_EQ(requests[7].target_state, LifecycleState::kActive);
  EXPECT_EQ(driver->state(), LifecycleState::kActive);
  manager.stop();
}

TEST(DeviceManager, DriverStateExceptionBecomesUnknownObservationAndErrorRecord)
{
  auto driver = std::make_shared<ThrowingStateDriver>();
  DeviceManager manager(registration_list(registration(driver)));
  manager.start();

  EXPECT_EQ(manager.device("camera")->state, LifecycleState::kUnknown);
  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)},
    RequestPriority::kNormal).accepted);
  ASSERT_TRUE(wait_until([&manager]() {
      const auto device = manager.device("camera");
      return device && device->last_transition.has_value();
    }));

  const auto record = manager.device("camera")->last_transition.value();
  EXPECT_EQ(record.result, TransitionResult::kError);
  EXPECT_EQ(record.actual_state, LifecycleState::kUnknown);
  manager.stop();
}

TEST(DeviceManager, DisabledDeviceStartsFinalized)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kFinalized);
  DeviceManager manager(registration_list(registration(driver, false)));
  manager.start();

  const auto device = manager.device("camera");
  ASSERT_TRUE(device.has_value());
  EXPECT_FALSE(device->enabled);
  EXPECT_EQ(device->state, LifecycleState::kFinalized);
  EXPECT_EQ(driver->request_count(), 0U);
  manager.stop();
}

TEST(DeviceManager, RuntimeMaterializationDefaultsFollowFinalizedState)
{
  FakeDriver finalized_driver(LifecycleState::kFinalized);
  EXPECT_FALSE(finalized_driver.materialized());
  EXPECT_EQ(finalized_driver.materialize({}).result, TransitionResult::kFailure);
  EXPECT_EQ(finalized_driver.dematerialize().result, TransitionResult::kFailure);

  FakeDriver unconfigured_driver(LifecycleState::kUnconfigured);
  EXPECT_TRUE(unconfigured_driver.materialized());
}

TEST(DeviceManager, DisabledDeviceCanBeEnabledAndMaterializedByHook)
{
  auto driver = std::make_shared<FakeDriver>(
    LifecycleState::kFinalized,
    [](const HookContext & context) {
      if (context.enabled && context.state == LifecycleState::kFinalized) {
        return std::vector<TransitionRequest>{
        transition(LifecycleState::kFinalized, LifecycleState::kUnconfigured)};
      }
      return std::vector<TransitionRequest>{};
    });
  DeviceManager manager(registration_list(registration(driver, false)));
  manager.start();

  ASSERT_TRUE(manager.patch_parameters(
    "camera", {{"device.enable", true}}, RequestPriority::kUrgent).accepted);
  manager.tick();
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 1U;}));

  EXPECT_EQ(driver->state(), LifecycleState::kUnconfigured);
  const auto device = manager.device("camera");
  ASSERT_TRUE(device.has_value());
  EXPECT_TRUE(device->enabled);
  manager.stop();
}

TEST(DeviceManager, DisabledDeviceAcceptsExplicitLifecycleRequests)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kFinalized);
  DeviceManager manager(registration_list(registration(driver, false)));
  manager.start();

  ASSERT_TRUE(manager.enqueue(
    "camera", {transition(LifecycleState::kFinalized, LifecycleState::kUnconfigured)},
    RequestPriority::kUrgent).accepted);
  ASSERT_TRUE(wait_until([&driver]() {return driver->request_count() == 1U;}));

  EXPECT_EQ(driver->state(), LifecycleState::kUnconfigured);
  EXPECT_FALSE(manager.device("camera")->enabled);
  manager.stop();
}

TEST(DeviceManager, AllowsDisabledDeviceThatIsNotFinalized)
{
  auto driver = std::make_shared<FakeDriver>(LifecycleState::kUnconfigured);
  EXPECT_NO_THROW(DeviceManager(registration_list(registration(driver, false))));
}

TEST(DeviceManager, DisabledDeviceStateCanBeUnavailable)
{
  EXPECT_NO_THROW(
    DeviceManager(registration_list(registration(std::make_shared<ThrowingStateDriver>(), false))));
}

}  // namespace
}  // namespace device_manager
