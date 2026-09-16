#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace device_manager
{

using Clock = std::chrono::system_clock;

enum class LifecycleState : std::uint8_t
{
  kUnknown = 0,
  kUnconfigured = 1,
  kInactive = 2,
  kActive = 3,
  kFinalized = 4,
  kConfiguring = 10,
  kCleaningUp = 11,
  kShuttingDown = 12,
  kActivating = 13,
  kDeactivating = 14,
  kErrorProcessing = 15,
};

enum class TransitionResult : std::uint8_t
{
  kSuccess = 0,
  kFailure = 1,
  kError = 2,
};

enum class RequestPriority : std::uint8_t
{
  kNormal = 0,
  kUrgent = 1,
};

enum class EventLevel : std::uint8_t
{
  kOk = 0,
  kWarn = 1,
  kError = 2,
  kStale = 3,
};

using ParameterValue = std::variant<
  bool, std::int64_t, double, std::string, std::vector<bool>,
  std::vector<std::int64_t>, std::vector<double>, std::vector<std::string>>;
using ParameterMap = std::map<std::string, ParameterValue>;

struct DeviceEvent
{
  Clock::time_point timestamp;
  EventLevel level{EventLevel::kOk};
  std::string code;
  std::string message;
  std::string source;
  LifecycleState target_state{LifecycleState::kUnknown};
  std::map<std::string, std::string> values;
};

struct TransitionRequest
{
  LifecycleState from_state{LifecycleState::kUnknown};
  LifecycleState target_state{LifecycleState::kUnknown};
  ParameterMap parameters;
};

struct TransitionOutcome
{
  TransitionResult result{TransitionResult::kError};
  std::string message;
};

struct TransitionRecord
{
  TransitionRequest request;
  TransitionResult result{TransitionResult::kError};
  bool discarded{false};
  LifecycleState actual_state{LifecycleState::kUnknown};
  std::string message;
  Clock::time_point timestamp;
};

struct HookContext
{
  Clock::time_point now;
  LifecycleState state{LifecycleState::kUnknown};
  bool enabled{false};
  ParameterMap parameters;
  std::optional<DeviceEvent> latest_event;
  std::uint64_t latest_event_revision{0};
};

using LifecycleHook = std::function<std::vector<TransitionRequest>(const HookContext &)>;
using EventHandler = std::function<void(DeviceEvent)>;
using HookRegistrar = std::function<void(LifecycleHook)>;

inline std::vector<TransitionRequest> standard_parameter_transitions(
  LifecycleState state, const ParameterMap & parameters)
{
  std::vector<TransitionRequest> requests;
  switch (state) {
    case LifecycleState::kActive:
      requests.push_back({LifecycleState::kActive, LifecycleState::kInactive, {}});
      [[fallthrough]];
    case LifecycleState::kInactive:
      requests.push_back({LifecycleState::kInactive, LifecycleState::kUnconfigured, {}});
      [[fallthrough]];
    case LifecycleState::kFinalized:
      if (state == LifecycleState::kFinalized) {
        requests.push_back({LifecycleState::kFinalized, LifecycleState::kUnconfigured, {}});
      }
      [[fallthrough]];
    case LifecycleState::kUnconfigured:
      requests.push_back(
        {LifecycleState::kUnconfigured, LifecycleState::kInactive, parameters});
      break;
    default:
      return {};
  }
  if (state == LifecycleState::kActive) {
    requests.push_back({LifecycleState::kInactive, LifecycleState::kActive, {}});
  }
  return requests;
}

struct DeviceDefinition;

class IDeviceRuntime
{
public:
  virtual ~IDeviceRuntime() = default;

  virtual void initialize(const DeviceDefinition &) {}
  virtual LifecycleState state() const = 0;
  virtual TransitionOutcome request_transition(const TransitionRequest & request) = 0;
  virtual std::vector<TransitionRequest> parameter_transitions(
    LifecycleState state, const ParameterMap & parameters, const ParameterMap & patch) = 0;
  virtual bool materialized() const
  {
    return state() != LifecycleState::kFinalized;
  }
  virtual TransitionOutcome materialize(const ParameterMap &)
  {
    return {TransitionResult::kFailure, "runtime does not support materialization"};
  }
  virtual TransitionOutcome dematerialize()
  {
    return {TransitionResult::kFailure, "runtime does not support dematerialization"};
  }
  virtual void set_event_handler(EventHandler handler) = 0;
  virtual void register_hook(HookRegistrar registrar) = 0;
};

class DeviceRuntimeBase : public IDeviceRuntime
{
public:
  TransitionOutcome request_transition(const TransitionRequest & request) override
  {
    if (request.from_state == LifecycleState::kFinalized &&
      request.target_state == LifecycleState::kUnconfigured)
    {
      return materialize(request.parameters);
    }
    if (request.from_state == LifecycleState::kUnconfigured &&
      request.target_state == LifecycleState::kInactive)
    {
      return configure(request.parameters);
    }
    if (request.from_state == LifecycleState::kInactive &&
      request.target_state == LifecycleState::kActive)
    {
      return activate();
    }
    if (request.from_state == LifecycleState::kActive &&
      request.target_state == LifecycleState::kInactive)
    {
      return deactivate();
    }
    if (request.from_state == LifecycleState::kInactive &&
      request.target_state == LifecycleState::kUnconfigured)
    {
      return cleanup(request.parameters);
    }
    if (request.target_state == LifecycleState::kFinalized &&
      (request.from_state == LifecycleState::kUnconfigured ||
      request.from_state == LifecycleState::kInactive ||
      request.from_state == LifecycleState::kActive))
    {
      return dematerialize();
    }
    return {TransitionResult::kFailure, "unsupported lifecycle transition"};
  }

protected:
  virtual TransitionOutcome configure(const ParameterMap &)
  {
    return {TransitionResult::kFailure, "runtime does not support configure"};
  }
  virtual TransitionOutcome activate()
  {
    return {TransitionResult::kFailure, "runtime does not support activate"};
  }
  virtual TransitionOutcome deactivate()
  {
    return {TransitionResult::kFailure, "runtime does not support deactivate"};
  }
  virtual TransitionOutcome cleanup(const ParameterMap &)
  {
    return {TransitionResult::kFailure, "runtime does not support cleanup"};
  }
};

struct DeviceDefinition
{
  std::string id;
  std::string type;
  bool enabled{true};
  ParameterMap parameters;
};

struct DeviceRegistration
{
  DeviceDefinition definition;
  std::shared_ptr<IDeviceRuntime> runtime;
};

struct DeviceObservation
{
  std::string id;
  std::string type;
  bool enabled{true};
  LifecycleState state{LifecycleState::kUnknown};
  ParameterMap parameters;
  std::optional<DeviceEvent> latest_event;
  std::optional<TransitionRecord> last_transition;
};

struct SubmissionResult
{
  bool accepted{false};
  std::string message;
};

class DeviceManager
{
public:
  explicit DeviceManager(std::vector<DeviceRegistration> registrations);
  ~DeviceManager();

  DeviceManager(const DeviceManager &) = delete;
  DeviceManager & operator=(const DeviceManager &) = delete;

  void start();
  void stop();

  SubmissionResult enqueue(
    const std::string & device_id, std::vector<TransitionRequest> requests,
    RequestPriority priority);
  SubmissionResult patch_parameters(
    const std::string & device_id, const ParameterMap & patch,
    RequestPriority priority);
  void tick();

  std::optional<DeviceObservation> device(const std::string & device_id) const;
  std::vector<DeviceObservation> devices() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace device_manager
