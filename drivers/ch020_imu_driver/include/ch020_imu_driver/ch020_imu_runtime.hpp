#ifndef CH020_IMU_DRIVER__CH020_IMU_RUNTIME_HPP_
#define CH020_IMU_DRIVER__CH020_IMU_RUNTIME_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "device_manager_core/device_manager.hpp"
#include "ch020_imu_driver/ch020_imu_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

namespace ch020_imu_driver
{

template<typename T>
class CircleMinTracker
{
public:
  explicit CircleMinTracker(int window)
  : window_(window)
  {
  }

  void push_back(T value)
  {
    if (count_ == 0) {
      min_value_ = value;
      min_index_ = 0;
      backup_ = value;
    } else if (value <= min_value_) {
      min_value_ = value;
      min_index_ = count_;
      backup_ = value;
    } else {
      if (value < backup_) {
        backup_ = value;
      }
      if (count_ - min_index_ >= static_cast<std::int64_t>(window_)) {
        min_value_ = backup_;
        min_index_ = count_;
        backup_ = value;
      }
    }
    ++count_;
  }

  T get_min() const {return min_value_;}
  void reset() {count_ = 0;}

private:
  int window_;
  T min_value_ {};
  T backup_ {};
  std::int64_t min_index_ {0};
  std::int64_t count_ {0};
};

class Ch020ImuRuntime final : public device_manager::DeviceRuntimeBase
{
public:
  explicit Ch020ImuRuntime(
    Ch020ImuConfig config = {},
    Ch020ImuDriver::TransportFactory transport_factory = {});

  void initialize(const device_manager::DeviceDefinition & definition) override;
  device_manager::LifecycleState state() const override;
  std::vector<device_manager::TransitionRequest> parameter_transitions(
    device_manager::LifecycleState state,
    const device_manager::ParameterMap & parameters,
    const device_manager::ParameterMap &) override
  {
    return device_manager::standard_parameter_transitions(state, parameters);
  }
  device_manager::TransitionOutcome materialize(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome dematerialize() override;
  void set_event_handler(device_manager::EventHandler handler) override;
  void register_hook(device_manager::HookRegistrar registrar) override;

  void set_packet_handler(Ch020ImuDriver::PacketHandler handler);
  Ch020ImuSnapshot snapshot() const;

private:
  device_manager::TransitionOutcome configure(
    const device_manager::ParameterMap & parameters) override;
  device_manager::TransitionOutcome activate() override;
  device_manager::TransitionOutcome deactivate() override;
  device_manager::TransitionOutcome cleanup(
    const device_manager::ParameterMap & parameters) override;

  Ch020ImuConfig config_from(
    const device_manager::ParameterMap & parameters,
    std::string & error_message) const;
  void emit_event(device_manager::DeviceEvent event);
  void emit_error_event(
    std::string code,
    std::string message,
    device_manager::LifecycleState target_state);
  void emit_ok_event(std::string code, std::string message);
  void publish_packet(const ImuSolPacket & packet);
  rclcpp::Time make_stamp(std::int64_t local_ns, std::uint32_t hardware_timestamp_ms);

  mutable std::mutex mutex_;
  Ch020ImuConfig config_;
  Ch020ImuDriver::TransportFactory transport_factory_;
  device_manager::LifecycleState state_;
  device_manager::EventHandler event_handler_;
  Ch020ImuDriver::PacketHandler packet_handler_;
  std::shared_ptr<rclcpp::Node> output_node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_publisher_;
  std::string frame_id_ {"imu_link"};
  bool use_hardware_time_ {false};
  CircleMinTracker<std::int64_t> time_sync_ {200};

  // Destroy the driver first so its read thread cannot outlive callback targets.
  std::unique_ptr<Ch020ImuDriver> driver_;
};

}  // namespace ch020_imu_driver

#endif  // CH020_IMU_DRIVER__CH020_IMU_RUNTIME_HPP_
