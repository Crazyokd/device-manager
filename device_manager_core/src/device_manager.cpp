#include "device_manager_core/device_manager.hpp"

#include <atomic>
#include <condition_variable>
#include <deque>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

namespace device_manager
{
namespace
{

struct RequestBatch
{
  std::vector<TransitionRequest> requests;
  std::optional<std::uint64_t> parameter_revision;
  ParameterMap parameter_patch;
};

class Device;

class DeviceQueue
{
public:
  explicit DeviceQueue(Device & device)
  : device_(device)
  {
  }

  void start();
  void stop() noexcept;
  SubmissionResult enqueue(
    std::vector<TransitionRequest> requests, RequestPriority priority);
  SubmissionResult enqueue_parameter_patch(
    const ParameterMap & patch, RequestPriority priority);

private:
  void run();
  void finalize();
  void execute_parameter_patch(std::uint64_t revision, const ParameterMap & patch);
  void record_error(std::string message);
  void record_discarded(const TransitionRequest & request, std::string message);
  bool execute(const TransitionRequest & request);

  Device & device_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::deque<RequestBatch> urgent_queue_;
  std::deque<RequestBatch> normal_queue_;
  ParameterMap pending_parameter_patch_;
  std::thread worker_;
  bool running_{false};
  bool stopping_{false};
};

class Device : public std::enable_shared_from_this<Device>
{
public:
  explicit Device(DeviceRegistration registration)
  : definition(std::move(registration.definition)), runtime(std::move(registration.runtime)),
    parameters(definition.parameters), queue_(*this)
  {
    runtime->initialize(definition);
  }

  void start()
  {
    const std::weak_ptr<Device> weak_self = shared_from_this();
    runtime->set_event_handler(
      [weak_self](DeviceEvent event) {
        if (const auto self = weak_self.lock()) {
          std::lock_guard<std::mutex> lock(self->observation_mutex);
          self->latest_event = std::move(event);
          ++self->latest_event_revision;
        }
      });
    bool register_lifecycle_hook;
    {
      std::lock_guard<std::mutex> lock(observation_mutex);
      register_lifecycle_hook = !hook_registered;
    }
    if (register_lifecycle_hook) {
      try {
        runtime->register_hook(
          [weak_self](LifecycleHook registered_hook) {
            if (const auto self = weak_self.lock()) {
              std::lock_guard<std::mutex> lock(self->observation_mutex);
              if (self->hook) {
                throw std::logic_error("a device may register only one lifecycle hook");
              }
              self->hook = std::move(registered_hook);
            }
          });
        std::lock_guard<std::mutex> lock(observation_mutex);
        hook_registered = true;
      } catch (...) {
        std::lock_guard<std::mutex> lock(observation_mutex);
        hook = {};
        throw;
      }
    }
    queue_.start();
  }

  void stop() noexcept
  {
    queue_.stop();
    try {
      runtime->set_event_handler({});
    } catch (...) {
    }
  }

  SubmissionResult enqueue(
    std::vector<TransitionRequest> requests, RequestPriority priority)
  {
    return queue_.enqueue(std::move(requests), priority);
  }

  SubmissionResult enqueue_parameter_patch(
    const ParameterMap & patch, RequestPriority priority)
  {
    return queue_.enqueue_parameter_patch(patch, priority);
  }

  HookContext hook_context() const
  {
    HookContext context;
    context.now = Clock::now();
    context.state = safe_state();
    std::lock_guard<std::mutex> lock(observation_mutex);
    context.enabled = definition.enabled;
    context.parameters = parameters;
    context.latest_event = latest_event;
    context.latest_event_revision = latest_event_revision;
    return context;
  }

  LifecycleHook lifecycle_hook() const
  {
    std::lock_guard<std::mutex> lock(observation_mutex);
    return hook;
  }

  DeviceObservation observation() const
  {
    DeviceObservation result;
    result.id = definition.id;
    result.type = definition.type;
    result.state = safe_state();
    std::lock_guard<std::mutex> lock(observation_mutex);
    result.enabled = definition.enabled;
    result.parameters = parameters;
    result.latest_event = latest_event;
    result.last_transition = last_transition;
    return result;
  }

  DeviceDefinition definition;
  std::shared_ptr<IDeviceRuntime> runtime;
  mutable std::mutex observation_mutex;
  ParameterMap parameters;
  std::uint64_t parameter_revision{0};
  std::optional<DeviceEvent> latest_event;
  std::uint64_t latest_event_revision{0};
  std::optional<TransitionRecord> last_transition;
  LifecycleHook hook;

private:
  friend class DeviceQueue;

  LifecycleState safe_state() const noexcept
  {
    try {
      return runtime->state();
    } catch (...) {
      return LifecycleState::kUnknown;
    }
  }

  bool hook_registered{false};
  DeviceQueue queue_;
};

void DeviceQueue::start()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) {
      return;
    }
    stopping_ = false;
    running_ = true;
  }
  worker_ = std::thread([this]() {run();});
}

void DeviceQueue::stop() noexcept
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return;
    }
    stopping_ = true;
    running_ = false;
    urgent_queue_.clear();
    normal_queue_.clear();
    pending_parameter_patch_.clear();
  }
  condition_.notify_one();
  if (worker_.joinable()) {
    worker_.join();
  }
}

SubmissionResult DeviceQueue::enqueue(
  std::vector<TransitionRequest> requests, RequestPriority priority)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || stopping_) {
    return {false, "device manager is not running"};
  }
  auto & queue = priority == RequestPriority::kUrgent ? urgent_queue_ : normal_queue_;
  queue.push_back(RequestBatch{std::move(requests), std::nullopt, {}});
  condition_.notify_one();
  return {true, "accepted"};
}

SubmissionResult DeviceQueue::enqueue_parameter_patch(
  const ParameterMap & patch, RequestPriority priority)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || stopping_) {
    return {false, "device manager is not running"};
  }
  const auto enable_patch = patch.find("device.enable");
  if (enable_patch != patch.end() && !std::holds_alternative<bool>(enable_patch->second)) {
    return {false, "device.enable must be a boolean"};
  }
  {
    std::lock_guard<std::mutex> observation_lock(device_.observation_mutex);
    for (const auto & parameter : patch) {
      device_.parameters.insert_or_assign(parameter.first, parameter.second);
    }
    if (enable_patch != patch.end()) {
      device_.definition.enabled = std::get<bool>(enable_patch->second);
    }
    if (!device_.definition.enabled) {
      ++device_.parameter_revision;
      pending_parameter_patch_.clear();
      return {true, "accepted"};
    }
    if (patch.size() == 1U && enable_patch != patch.end()) {
      return {true, "accepted"};
    }
    ++device_.parameter_revision;
    for (const auto & parameter : patch) {
      pending_parameter_patch_.insert_or_assign(parameter.first, parameter.second);
    }
    auto & queue = priority == RequestPriority::kUrgent ? urgent_queue_ : normal_queue_;
    queue.push_back(RequestBatch{{}, device_.parameter_revision, pending_parameter_patch_});
  }
  condition_.notify_one();
  return {true, "accepted"};
}

void DeviceQueue::run()
{
  while (true) {
    RequestBatch batch;
    bool finalizing = false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      condition_.wait(
        lock, [this]() {
          return stopping_ || !urgent_queue_.empty() || !normal_queue_.empty();
        });
      if (stopping_) {
        finalizing = true;
      } else {
        auto & queue = urgent_queue_.empty() ? normal_queue_ : urgent_queue_;
        batch = std::move(queue.front());
        queue.pop_front();
      }
    }
    if (finalizing) {
      finalize();
      return;
    }

    if (batch.parameter_revision) {
      execute_parameter_patch(*batch.parameter_revision, batch.parameter_patch);
      continue;
    }
    for (const auto & request : batch.requests) {
      if (!execute(request)) {
        break;
      }
    }
  }
}

void DeviceQueue::finalize()
{
  const auto state = device_.safe_state();
  if (state == LifecycleState::kFinalized) {
    return;
  }
  (void)execute({state, LifecycleState::kFinalized, {}});
}

void DeviceQueue::execute_parameter_patch(std::uint64_t revision, const ParameterMap & patch)
{
  ParameterMap current_parameters;
  bool superseded;
  {
    std::lock_guard<std::mutex> queue_lock(mutex_);
    std::lock_guard<std::mutex> observation_lock(device_.observation_mutex);
    superseded = revision != device_.parameter_revision;
    current_parameters = device_.parameters;
    if (!superseded) {
      pending_parameter_patch_.clear();
    }
  }
  if (superseded) {
    record_discarded({}, "superseded parameter patch");
    return;
  }

  LifecycleState initial_state;
  try {
    initial_state = device_.runtime->state();
  } catch (const std::exception & exception) {
    record_error(exception.what());
    return;
  } catch (...) {
    record_error("runtime state query threw an unknown exception");
    return;
  }

  try {
    for (const auto & request :
      device_.runtime->parameter_transitions(initial_state, current_parameters, patch))
    {
      if (!execute(request)) {
        return;
      }
    }
  } catch (const std::exception & exception) {
    record_error(exception.what());
  } catch (...) {
    record_error("runtime parameter transition planning threw an unknown exception");
  }
}

void DeviceQueue::record_error(std::string message)
{
  TransitionRecord record;
  record.result = TransitionResult::kError;
  record.actual_state = LifecycleState::kUnknown;
  record.message = std::move(message);
  record.timestamp = Clock::now();
  std::lock_guard<std::mutex> lock(device_.observation_mutex);
  device_.last_transition = std::move(record);
}

void DeviceQueue::record_discarded(
  const TransitionRequest & request, std::string message)
{
  TransitionRecord record;
  record.request = request;
  record.result = TransitionResult::kFailure;
  record.discarded = true;
  record.actual_state = device_.safe_state();
  record.message = std::move(message);
  record.timestamp = Clock::now();
  std::lock_guard<std::mutex> lock(device_.observation_mutex);
  device_.last_transition = std::move(record);
}

bool DeviceQueue::execute(const TransitionRequest & request)
{
  TransitionRecord record;
  record.request = request;

  LifecycleState source_state;
  try {
    source_state = device_.runtime->state();
  } catch (const std::exception & exception) {
    record.result = TransitionResult::kError;
    record.actual_state = LifecycleState::kUnknown;
    record.message = exception.what();
    record.timestamp = Clock::now();
    std::lock_guard<std::mutex> lock(device_.observation_mutex);
    device_.last_transition = std::move(record);
    return false;
  } catch (...) {
    record.result = TransitionResult::kError;
    record.actual_state = LifecycleState::kUnknown;
    record.message = "runtime state query threw an unknown exception";
    record.timestamp = Clock::now();
    std::lock_guard<std::mutex> lock(device_.observation_mutex);
    device_.last_transition = std::move(record);
    return false;
  }

  if (source_state != request.from_state) {
    record.result = TransitionResult::kFailure;
    record.discarded = true;
    record.actual_state = source_state;
    record.message = "discarded because the source state is stale";
  } else {
    try {
      const auto outcome = device_.runtime->request_transition(request);
      record.result = outcome.result;
      record.message = outcome.message;
      record.actual_state = device_.runtime->state();
    } catch (const std::exception & exception) {
      record.result = TransitionResult::kError;
      record.actual_state = device_.safe_state();
      record.message = exception.what();
    } catch (...) {
      record.result = TransitionResult::kError;
      record.actual_state = device_.safe_state();
      record.message = "runtime transition threw an unknown exception";
    }
  }
  record.timestamp = Clock::now();

  if (record.result == TransitionResult::kSuccess &&
    record.actual_state != request.target_state)
  {
    record.result = TransitionResult::kFailure;
    record.message = "runtime reported success without reaching the target state";
  }

  const bool succeeded =
    record.result == TransitionResult::kSuccess &&
    record.actual_state == request.target_state;
  std::lock_guard<std::mutex> lock(device_.observation_mutex);
  device_.last_transition = std::move(record);
  return succeeded;
}

bool is_contiguous(const std::vector<TransitionRequest> & requests)
{
  for (std::size_t index = 1; index < requests.size(); ++index) {
    if (requests[index - 1].target_state != requests[index].from_state) {
      return false;
    }
  }
  return true;
}

}  // namespace

class DeviceManager::Impl
{
public:
  explicit Impl(std::vector<DeviceRegistration> registrations)
  {
    for (auto & registration : registrations) {
      if (registration.definition.id.empty()) {
        throw std::invalid_argument("device id must not be empty");
      }
      if (!registration.runtime) {
        throw std::invalid_argument("device runtime must not be null");
      }
      auto device = std::make_shared<Device>(std::move(registration));
      if (!devices.emplace(device->definition.id, std::move(device)).second) {
        throw std::invalid_argument("duplicate device id");
      }
    }
  }

  std::unordered_map<std::string, std::shared_ptr<Device>> devices;
  std::mutex lifecycle_mutex;
  std::mutex submission_mutex;
  std::atomic_bool running{false};
};

DeviceManager::DeviceManager(std::vector<DeviceRegistration> registrations)
: impl_(std::make_unique<Impl>(std::move(registrations)))
{
}

DeviceManager::~DeviceManager()
{
  stop();
}

void DeviceManager::start()
{
  std::lock_guard<std::mutex> lock(impl_->lifecycle_mutex);
  if (impl_->running.load()) {
    return;
  }
  try {
    for (const auto & entry : impl_->devices) {
      entry.second->start();
    }
    std::lock_guard<std::mutex> submission_lock(impl_->submission_mutex);
    impl_->running.store(true);
  } catch (...) {
    for (const auto & entry : impl_->devices) {
      entry.second->stop();
    }
    throw;
  }
}

void DeviceManager::stop()
{
  if (!impl_) {
    return;
  }
  std::lock_guard<std::mutex> lock(impl_->lifecycle_mutex);
  {
    std::lock_guard<std::mutex> submission_lock(impl_->submission_mutex);
    if (!impl_->running.exchange(false)) {
      return;
    }
  }
  for (const auto & entry : impl_->devices) {
    entry.second->stop();
  }
}

SubmissionResult DeviceManager::enqueue(
  const std::string & device_id, std::vector<TransitionRequest> requests,
  RequestPriority priority)
{
  std::lock_guard<std::mutex> lock(impl_->submission_mutex);
  if (!impl_->running.load()) {
    return {false, "device manager is not running"};
  }
  const auto found = impl_->devices.find(device_id);
  if (found == impl_->devices.end()) {
    return {false, "unknown device"};
  }
  if (requests.empty()) {
    return {false, "transition request batch must not be empty"};
  }
  if (!is_contiguous(requests)) {
    return {false, "transition request batch is not contiguous"};
  }
  return found->second->enqueue(std::move(requests), priority);
}

SubmissionResult DeviceManager::patch_parameters(
  const std::string & device_id, const ParameterMap & patch,
  RequestPriority priority)
{
  std::lock_guard<std::mutex> lock(impl_->submission_mutex);
  if (!impl_->running.load()) {
    return {false, "device manager is not running"};
  }
  const auto found = impl_->devices.find(device_id);
  if (found == impl_->devices.end()) {
    return {false, "unknown device"};
  }
  const auto & device = found->second;
  if (patch.empty()) {
    return {false, "parameter patch must not be empty"};
  }

  return device->enqueue_parameter_patch(patch, priority);
}

void DeviceManager::tick()
{
  if (!impl_->running.load()) {
    return;
  }
  for (const auto & entry : impl_->devices) {
    try {
      const auto & device = entry.second;
      const auto hook = device->lifecycle_hook();
      if (!hook) {
        continue;
      }
      auto requests = hook(device->hook_context());
      if (!requests.empty()) {
        enqueue(device->definition.id, std::move(requests), RequestPriority::kNormal);
      }
    } catch (const std::exception & exception) {
      std::cerr << "device_manager hook failed for device "
                << entry.first << ": " << exception.what() << '\n';
    } catch (...) {
      std::cerr << "device_manager hook failed for device "
                << entry.first << ": unknown exception\n";
    }
  }
}

std::optional<DeviceObservation> DeviceManager::device(const std::string & device_id) const
{
  const auto found = impl_->devices.find(device_id);
  if (found == impl_->devices.end()) {
    return std::nullopt;
  }
  return found->second->observation();
}

std::vector<DeviceObservation> DeviceManager::devices() const
{
  std::vector<DeviceObservation> result;
  result.reserve(impl_->devices.size());
  for (const auto & entry : impl_->devices) {
    result.push_back(entry.second->observation());
  }
  return result;
}

}  // namespace device_manager
