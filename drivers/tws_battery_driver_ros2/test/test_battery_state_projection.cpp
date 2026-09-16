#include <gtest/gtest.h>

#include "sensor_msgs/msg/battery_state.hpp"
#include "tws_battery_driver_ros2/battery_state_projection.hpp"

namespace tws_battery_driver_ros2
{
namespace
{

TEST(BatteryStateProjection, PublishesIdleAndNotChargingWhenOffline)
{
  const auto projection = project_battery_state(false, 1U, 50.0F);

  EXPECT_EQ(projection.work_state, 0U);
  EXPECT_EQ(
    projection.power_supply_status,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING);
}

TEST(BatteryStateProjection, OfflineFullBatteryIsStillNotCharging)
{
  const auto projection = project_battery_state(false, 1U, 100.0F);

  EXPECT_EQ(projection.work_state, 0U);
  EXPECT_EQ(
    projection.power_supply_status,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING);
}

TEST(BatteryStateProjection, PreservesOnlineStateMapping)
{
  EXPECT_EQ(
    project_battery_state(true, 1U, 50.0F).power_supply_status,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
  EXPECT_EQ(
    project_battery_state(true, 2U, 50.0F).power_supply_status,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
  EXPECT_EQ(
    project_battery_state(true, 0U, 99.0F).power_supply_status,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL);
  EXPECT_EQ(
    project_battery_state(true, 0U, 50.0F).power_supply_status,
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN);
  EXPECT_EQ(project_battery_state(true, 4U, 50.0F).work_state, 4U);
}

}  // namespace
}  // namespace tws_battery_driver_ros2
