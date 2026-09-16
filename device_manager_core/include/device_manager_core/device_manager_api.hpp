#pragma once

#include "device_manager_core/device_manager_application.hpp"

namespace device_manager
{

class DeviceManagerApi
{
public:
  explicit DeviceManagerApi(IDeviceManagerApplication & application)
  : application_(application)
  {
  }
  virtual ~DeviceManagerApi() = default;

  DeviceManagerApi(const DeviceManagerApi &) = delete;
  DeviceManagerApi & operator=(const DeviceManagerApi &) = delete;

  virtual void start() = 0;
  virtual void stop() = 0;

protected:
  IDeviceManagerApplication & application()
  {
    return application_;
  }

private:
  IDeviceManagerApplication & application_;
};

}  // namespace device_manager
