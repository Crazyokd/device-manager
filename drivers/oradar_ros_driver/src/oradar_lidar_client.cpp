#include "oradar_ros_driver/oradar_lidar_client.hpp"

#include <arpa/inet.h>

#include <memory>
#include <stdexcept>
#include <string>

#include "ord/lidar_address.h"
#include "ord/ord_driver.h"

namespace oradar_ros_driver
{
namespace
{

class OrdDriverClient final : public OradarLidarClient
{
public:
  explicit OrdDriverClient(const OradarLidarConfig & config)
  {
    const in_addr_t address = ::inet_addr(config.ip_address.c_str());
    if (address == htonl(INADDR_NONE)) {
      throw std::invalid_argument("invalid lidar ip address: " + config.ip_address);
    }
    const in_port_t port = htons(static_cast<std::uint16_t>(config.udp_port));
    ord_sdk::LidarAddress location(address, port, config.network_interface);
    driver_ = std::make_unique<ord_sdk::OrdDriver>(location);
    driver_->setTimeout(config.timeout_ms);
  }

  ord_sdk::error_t open() override {return driver_->open();}
  bool is_open() const override {return driver_->isOpened();}
  void close() override {driver_->close();}
  void set_timeout(int timeout_ms) override {driver_->setTimeout(timeout_ms);}
  ord_sdk::error_t track_connect() override {return driver_->trackConnect();}
  ord_sdk::error_t enable_measure() override {return driver_->enableMeasure();}
  ord_sdk::error_t disable_measure() override {return driver_->disableMeasure();}
  ord_sdk::error_t enable_data_stream() override {return driver_->enabelDataStream();}
  ord_sdk::error_t disable_data_stream() override {return driver_->disableDataStream();}
  ord_sdk::error_t get_scan_speed(std::uint32_t & speed) override
  {
    return driver_->getScanSpeed(speed);
  }
  ord_sdk::error_t set_scan_speed(std::uint32_t speed) override
  {
    return driver_->setScanSpeed(speed);
  }
  ord_sdk::error_t get_tail_filter_level(std::uint32_t & level) override
  {
    return driver_->getTailFilterLevel(level);
  }
  ord_sdk::error_t set_tail_filter_level(std::uint32_t level) override
  {
    return driver_->setTailFilterLevel(level);
  }
  ord_sdk::error_t get_scan_direction(std::uint32_t & direction) override
  {
    return driver_->getScanDirection(direction);
  }
  ord_sdk::error_t set_scan_direction(std::uint32_t direction) override
  {
    return driver_->setScanDirection(direction);
  }
  ord_sdk::error_t apply_configs() override {return driver_->applyConfigs();}
  ord_sdk::error_t get_scan_frame_data(ord_sdk::ScanFrameData & frame) override
  {
    return driver_->getScanFrameData(frame);
  }

private:
  std::unique_ptr<ord_sdk::OrdDriver> driver_;
};

}  // namespace

std::unique_ptr<OradarLidarClient> make_ord_driver_client(const OradarLidarConfig & config)
{
  return std::make_unique<OrdDriverClient>(config);
}

}  // namespace oradar_ros_driver
