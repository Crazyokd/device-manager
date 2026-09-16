#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "in_process_driver.hpp"

namespace device_manager
{
namespace
{

TransitionRequest transition(LifecycleState from, LifecycleState target)
{
  return TransitionRequest{from, target, {}};
}

class DispatchRuntime final : public DeviceRuntimeBase
{
public:
  LifecycleState state() const override
  {
    return state_;
  }

  std::vector<TransitionRequest> parameter_transitions(
    LifecycleState, const ParameterMap &, const ParameterMap &) override
  {
    return {};
  }

  void set_event_handler(EventHandler) override {}
  void register_hook(HookRegistrar) override {}

  const std::vector<std::string> & calls() const
  {
    return calls_;
  }

private:
  TransitionOutcome materialize(const ParameterMap &) override
  {
    calls_.push_back("materialize");
    state_ = LifecycleState::kUnconfigured;
    return {TransitionResult::kSuccess, {}};
  }

  TransitionOutcome configure(const ParameterMap &) override
  {
    calls_.push_back("configure");
    state_ = LifecycleState::kInactive;
    return {TransitionResult::kSuccess, {}};
  }

  TransitionOutcome activate() override
  {
    calls_.push_back("activate");
    state_ = LifecycleState::kActive;
    return {TransitionResult::kSuccess, {}};
  }

  TransitionOutcome deactivate() override
  {
    calls_.push_back("deactivate");
    state_ = LifecycleState::kInactive;
    return {TransitionResult::kSuccess, {}};
  }

  TransitionOutcome cleanup(const ParameterMap &) override
  {
    calls_.push_back("cleanup");
    state_ = LifecycleState::kUnconfigured;
    return {TransitionResult::kSuccess, {}};
  }

  TransitionOutcome dematerialize() override
  {
    calls_.push_back("dematerialize");
    state_ = LifecycleState::kFinalized;
    return {TransitionResult::kSuccess, {}};
  }

  LifecycleState state_{LifecycleState::kFinalized};
  std::vector<std::string> calls_;
};

TEST(InProcessDeviceDriver, MapsTheStandardLifecycleEdges)
{
  InProcessDeviceDriver driver(LifecycleState::kUnconfigured);

  EXPECT_EQ(
    driver.request_transition(
      transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)).result,
    TransitionResult::kSuccess);
  EXPECT_EQ(driver.state(), LifecycleState::kInactive);
  EXPECT_EQ(
    driver.request_transition(
      transition(LifecycleState::kInactive, LifecycleState::kActive)).result,
    TransitionResult::kSuccess);
  EXPECT_EQ(driver.state(), LifecycleState::kActive);
  EXPECT_EQ(
    driver.request_transition(
      transition(LifecycleState::kActive, LifecycleState::kInactive)).result,
    TransitionResult::kSuccess);
  EXPECT_EQ(driver.state(), LifecycleState::kInactive);
  EXPECT_EQ(
    driver.request_transition(
      transition(LifecycleState::kInactive, LifecycleState::kUnconfigured)).result,
    TransitionResult::kSuccess);
  EXPECT_EQ(driver.state(), LifecycleState::kUnconfigured);
  EXPECT_EQ(
    driver.request_transition(
      transition(LifecycleState::kUnconfigured, LifecycleState::kFinalized)).result,
    TransitionResult::kSuccess);
  EXPECT_EQ(driver.state(), LifecycleState::kFinalized);
  EXPECT_EQ(
    driver.request_transition(
      transition(LifecycleState::kFinalized, LifecycleState::kUnconfigured)).result,
    TransitionResult::kSuccess);
  EXPECT_EQ(driver.state(), LifecycleState::kUnconfigured);
}

TEST(DeviceRuntimeBase, DispatchesLifecycleEdgesToSpecificMethods)
{
  DispatchRuntime runtime;

  EXPECT_EQ(
    runtime.request_transition(
      transition(LifecycleState::kFinalized, LifecycleState::kUnconfigured)).result,
    TransitionResult::kSuccess);
  EXPECT_EQ(
    runtime.request_transition(
      transition(LifecycleState::kUnconfigured, LifecycleState::kInactive)).result,
    TransitionResult::kSuccess);
  EXPECT_EQ(
    runtime.request_transition(
      transition(LifecycleState::kInactive, LifecycleState::kActive)).result,
    TransitionResult::kSuccess);
  EXPECT_EQ(
    runtime.request_transition(
      transition(LifecycleState::kActive, LifecycleState::kInactive)).result,
    TransitionResult::kSuccess);
  EXPECT_EQ(
    runtime.request_transition(
      transition(LifecycleState::kInactive, LifecycleState::kUnconfigured)).result,
    TransitionResult::kSuccess);
  EXPECT_EQ(
    runtime.request_transition(
      transition(LifecycleState::kUnconfigured, LifecycleState::kFinalized)).result,
    TransitionResult::kSuccess);

  EXPECT_EQ(
    runtime.calls(),
    (std::vector<std::string>{
        "materialize", "configure", "activate", "deactivate", "cleanup",
        "dematerialize"}));
}

TEST(InProcessDeviceDriver, RejectsAnInvalidLifecycleEdge)
{
  InProcessDeviceDriver driver(LifecycleState::kUnconfigured);

  const auto outcome = driver.request_transition(
    transition(LifecycleState::kUnconfigured, LifecycleState::kActive));

  EXPECT_EQ(outcome.result, TransitionResult::kFailure);
  EXPECT_EQ(driver.state(), LifecycleState::kUnconfigured);
}

TEST(InProcessDeviceDriver, ChangesStateOnlyWhenItsHandlerSucceeds)
{
  std::vector<TransitionResult> results{
    TransitionResult::kFailure, TransitionResult::kError, TransitionResult::kSuccess};
  InProcessDeviceDriver driver(
    LifecycleState::kUnconfigured,
    [&results](const TransitionRequest &) {
      const auto result = results.front();
      results.erase(results.begin());
      return TransitionOutcome{result, "handled"};
    });
  const auto request = transition(LifecycleState::kUnconfigured, LifecycleState::kInactive);

  EXPECT_EQ(driver.request_transition(request).result, TransitionResult::kFailure);
  EXPECT_EQ(driver.state(), LifecycleState::kUnconfigured);
  EXPECT_EQ(driver.request_transition(request).result, TransitionResult::kError);
  EXPECT_EQ(driver.state(), LifecycleState::kUnconfigured);
  EXPECT_EQ(driver.request_transition(request).result, TransitionResult::kSuccess);
  EXPECT_EQ(driver.state(), LifecycleState::kInactive);
}

}  // namespace
}  // namespace device_manager
