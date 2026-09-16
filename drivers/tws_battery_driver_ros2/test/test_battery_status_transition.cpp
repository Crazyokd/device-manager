#include <gtest/gtest.h>

#include "tws_battery_driver_ros2/battery_soc_transition_tracker.hpp"
#include "tws_battery_driver_ros2/battery_status_transition_log.hpp"
#include "tws_battery_driver_ros2/battery_status_transition_tracker.hpp"

namespace tws_battery_driver_ros2
{
namespace
{

TEST(BatteryStatusTransitionTracker, ReportsFirstSuccessfulPollAsOnlineOnce)
{
  BatteryStatusTransitionTracker tracker;

  const auto event = tracker.observe_online(0U);

  ASSERT_TRUE(event.has_value());
  EXPECT_EQ(event->type, BatteryStatusTransitionType::kOnline);
  EXPECT_EQ(event->current_work_state, 0U);
  EXPECT_FALSE(tracker.observe_online(0U).has_value());
}

TEST(BatteryStatusTransitionTracker, ReportsEveryWorkStateValueChange)
{
  BatteryStatusTransitionTracker tracker;
  ASSERT_TRUE(tracker.observe_online(0U).has_value());

  uint16_t previous = 0U;
  for (const uint16_t current : {1U, 2U, 3U, 4U, 5U, 99U}) {
    const auto event = tracker.observe_online(current);

    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, BatteryStatusTransitionType::kWorkStateChanged);
    EXPECT_EQ(event->previous_work_state, previous);
    EXPECT_EQ(event->current_work_state, current);
    previous = current;
  }
}

TEST(BatteryStatusTransitionTracker, IgnoresFailuresBeforeFirstOnline)
{
  BatteryStatusTransitionTracker tracker;

  EXPECT_FALSE(tracker.observe_offline("等待 BMS 响应超时").has_value());
}

TEST(BatteryStatusTransitionTracker, ReportsOnlyFirstFailureAfterOnline)
{
  BatteryStatusTransitionTracker tracker;
  ASSERT_TRUE(tracker.observe_online(1U).has_value());

  const auto event = tracker.observe_offline("等待 BMS 响应超时");

  ASSERT_TRUE(event.has_value());
  EXPECT_EQ(event->type, BatteryStatusTransitionType::kOffline);
  EXPECT_EQ(event->error_message, "等待 BMS 响应超时");
  EXPECT_FALSE(tracker.observe_offline("再次超时").has_value());
}

TEST(BatteryStatusTransitionTracker, ReportsReconnectAsOnlineWithoutStateChange)
{
  BatteryStatusTransitionTracker tracker;
  ASSERT_TRUE(tracker.observe_online(1U).has_value());
  ASSERT_TRUE(tracker.observe_offline("等待 BMS 响应超时").has_value());

  const auto event = tracker.observe_online(2U);

  ASSERT_TRUE(event.has_value());
  EXPECT_EQ(event->type, BatteryStatusTransitionType::kOnline);
  EXPECT_EQ(event->current_work_state, 2U);
  EXPECT_FALSE(tracker.observe_online(2U).has_value());
}

TEST(BatterySocTransitionTracker, FirstAndRepeatedSocDoNotProduceEvents)
{
  BatterySocTransitionTracker tracker;

  EXPECT_FALSE(tracker.observe(21.0F).has_value());
  EXPECT_FALSE(tracker.observe(21.0F).has_value());
}

TEST(BatterySocTransitionTracker, ReportsEveryIncreaseAndDecrease)
{
  BatterySocTransitionTracker tracker;
  ASSERT_FALSE(tracker.observe(21.0F).has_value());

  const auto increase = tracker.observe(22.0F);
  ASSERT_TRUE(increase.has_value());
  EXPECT_FLOAT_EQ(increase->previous_soc, 21.0F);
  EXPECT_FLOAT_EQ(increase->current_soc, 22.0F);

  const auto decrease = tracker.observe(20.0F);
  ASSERT_TRUE(decrease.has_value());
  EXPECT_FLOAT_EQ(decrease->previous_soc, 22.0F);
  EXPECT_FLOAT_EQ(decrease->current_soc, 20.0F);
}

TEST(BatterySocTransitionTracker, ComparesReconnectSampleWithPreOfflineBaseline)
{
  BatterySocTransitionTracker tracker;
  ASSERT_FALSE(tracker.observe(20.0F).has_value());

  const auto reconnect = tracker.observe(23.0F);

  ASSERT_TRUE(reconnect.has_value());
  EXPECT_FLOAT_EQ(reconnect->previous_soc, 20.0F);
  EXPECT_FLOAT_EQ(reconnect->current_soc, 23.0F);
}

TEST(BatteryStatusTransitionLog, MapsKnownAndUnknownWorkStates)
{
  EXPECT_EQ(battery_work_state_to_text(0U), "EM_IDLE");
  EXPECT_EQ(battery_work_state_to_text(1U), "EM_CHARGE");
  EXPECT_EQ(battery_work_state_to_text(2U), "EM_DISCHARGE");
  EXPECT_EQ(battery_work_state_to_text(3U), "EM_SLEEP");
  EXPECT_EQ(battery_work_state_to_text(4U), "EM_PROTECT");
  EXPECT_EQ(battery_work_state_to_text(5U), "EM_SHUTDOWN");
  EXPECT_EQ(battery_work_state_to_text(99U), "UNKNOWN");
}

TEST(BatteryStatusTransitionLog, FormatsOnlineEventAtInfoLevel)
{
  const BatteryStatusTransitionEvent event {
    BatteryStatusTransitionType::kOnline, 1U, 1U, ""};

  const auto record = make_battery_status_log_record(event);

  EXPECT_EQ(record.level, BatteryStatusLogLevel::kInfo);
  EXPECT_EQ(record.message, "电池上线: work_state=EM_CHARGE(1)");
}

TEST(BatteryStatusTransitionLog, FormatsWorkStateChangeAtInfoLevel)
{
  const BatteryStatusTransitionEvent event {
    BatteryStatusTransitionType::kWorkStateChanged, 1U, 2U, ""};

  const auto record = make_battery_status_log_record(event);

  EXPECT_EQ(record.level, BatteryStatusLogLevel::kInfo);
  EXPECT_EQ(
    record.message,
    "电池工作状态变化: EM_CHARGE(1) -> EM_DISCHARGE(2)");
}

TEST(BatteryStatusTransitionLog, FormatsOfflineEventAtWarnLevelWithReason)
{
  const BatteryStatusTransitionEvent event {
    BatteryStatusTransitionType::kOffline, 2U, 2U, "等待 BMS 响应超时"};

  const auto record = make_battery_status_log_record(event);

  EXPECT_EQ(record.level, BatteryStatusLogLevel::kWarn);
  EXPECT_EQ(record.message, "电池掉线: 等待 BMS 响应超时");
}

TEST(BatterySocTransitionLog, FormatsChangeAtInfoLevelWithOneDecimal)
{
  const BatterySocTransitionEvent event {21.0F, 22.5F};

  const auto record = make_battery_soc_log_record(event);

  EXPECT_EQ(record.level, BatteryStatusLogLevel::kInfo);
  EXPECT_EQ(record.message, "电池电量变化: 21.0% -> 22.5%");
}

}  // namespace
}  // namespace tws_battery_driver_ros2
