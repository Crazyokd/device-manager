#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "device_manager_core/device_manager.hpp"

namespace device_manager
{

class IDeviceManagerApplication
{
public:
  virtual ~IDeviceManagerApplication() = default;

  virtual std::optional<DeviceObservation> device(const std::string & id) const = 0;
  virtual std::vector<DeviceObservation> devices() const = 0;
  virtual SubmissionResult change_state(
    const std::string & id, std::vector<TransitionRequest> requests,
    RequestPriority priority) = 0;
  virtual SubmissionResult patch_parameters(
    const std::string & id, const ParameterMap & patch,
    RequestPriority priority) = 0;
};

class DeviceManagerApplication final : public IDeviceManagerApplication
{
public:
  DeviceManagerApplication(
    DeviceManager & manager, std::chrono::milliseconds tick_period);
  ~DeviceManagerApplication() override;

  DeviceManagerApplication(const DeviceManagerApplication &) = delete;
  DeviceManagerApplication & operator=(const DeviceManagerApplication &) = delete;

  void start();
  void stop();

  std::optional<DeviceObservation> device(const std::string & id) const override;
  std::vector<DeviceObservation> devices() const override;
  SubmissionResult change_state(
    const std::string & id, std::vector<TransitionRequest> requests,
    RequestPriority priority) override;
  SubmissionResult patch_parameters(
    const std::string & id, const ParameterMap & patch,
    RequestPriority priority) override;

private:
  void run();

  DeviceManager & manager_;
  std::chrono::milliseconds tick_period_;
  std::mutex lifecycle_mutex_;
  std::mutex mutex_;
  std::condition_variable condition_;
  std::thread tick_thread_;
  bool running_{false};
};

}  // namespace device_manager
