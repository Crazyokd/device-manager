#include "ord/lidar_address.h"

#include <utility>

namespace ord_sdk
{

LidarAddress::LidarAddress(in_addr_t address, in_port_t port)
  : address_(address)
  , port_(port)
{
}

LidarAddress::LidarAddress(in_addr_t address, in_port_t port, std::string interface_name)
  : address_(address)
  , port_(port)
  , interface_name_(std::move(interface_name))
{
}

bool LidarAddress::operator==(const LidarAddress& other) const
{
  return (address_ == other.address_) && (port_ == other.port_) &&
         (interface_name_ == other.interface_name_);
}

in_addr_t LidarAddress::address() const
{
  return address_;
}

in_port_t LidarAddress::port() const
{
  return port_;
}

const std::string& LidarAddress::interface_name() const
{
  return interface_name_;
}

}
