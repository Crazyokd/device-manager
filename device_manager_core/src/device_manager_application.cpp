#include "device_manager_core/device_manager_application.hpp"

#include <stdexcept>
#include <utility>

namespace device_manager
{

DeviceManagerApplication::DeviceManagerApplication(
  DeviceManager & manager, std::chrono::milliseconds tick_period)
: manager_(manager), tick_period_(tick_period)
{
  if (tick_period_ <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("tick period must be positive");
  }
}

DeviceManagerApplication::~DeviceManagerApplication()
{
  stop();
}

void DeviceManagerApplication::start()
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_) {
    return;
  }

  manager_.start();
  running_ = true;
  try {
    tick_thread_ = std::thread([this]() {run();});
  } catch (...) {
    running_ = false;
    manager_.stop();
    throw;
  }
}

void DeviceManagerApplication::stop()
{
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
      return;
    }
    running_ = false;
  }
  condition_.notify_one();
  manager_.stop();
  if (tick_thread_.joinable()) {
    tick_thread_.join();
  }
}

std::optional<DeviceObservation> DeviceManagerApplication::device(
  const std::string & id) const
{
  return manager_.device(id);
}

std::vector<DeviceObservation> DeviceManagerApplication::devices() const
{
  return manager_.devices();
}

SubmissionResult DeviceManagerApplication::change_state(
  const std::string & id, std::vector<TransitionRequest> requests,
  RequestPriority priority)
{
  return manager_.enqueue(id, std::move(requests), priority);
}

SubmissionResult DeviceManagerApplication::patch_parameters(
  const std::string & id, const ParameterMap & patch,
  RequestPriority priority)
{
  return manager_.patch_parameters(id, patch, priority);
}

void DeviceManagerApplication::run()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (running_) {
    if (condition_.wait_for(lock, tick_period_, [this]() {return !running_;})) {
      return;
    }
    lock.unlock();
    manager_.tick();
    lock.lock();
  }
}

}  // namespace device_manager
