#include <gtest/gtest.h>

#include "tws_battery_driver_ros2/soc_spike_filter.hpp"

namespace tws_battery_driver_ros2
{
namespace
{

TEST(SocSpikeFilter, HoldsTransientFullSocSpikeUntilItRecovers)
{
  SocSpikeFilter filter;

  EXPECT_FLOAT_EQ(filter.filter(56.0F), 56.0F);
  EXPECT_FLOAT_EQ(filter.filter(100.0F), 56.0F);
  EXPECT_FLOAT_EQ(filter.filter(55.0F), 55.0F);
}

TEST(SocSpikeFilter, AcceptsFullSocAfterConsecutiveConfirmations)
{
  SocSpikeFilter filter;

  EXPECT_FLOAT_EQ(filter.filter(56.0F), 56.0F);
  EXPECT_FLOAT_EQ(filter.filter(100.0F), 56.0F);
  EXPECT_FLOAT_EQ(filter.filter(100.0F), 56.0F);
  EXPECT_FLOAT_EQ(filter.filter(100.0F), 100.0F);
}

TEST(SocSpikeFilter, AcceptsStartupAndNearFullTransitions)
{
  SocSpikeFilter startup_filter;
  EXPECT_FLOAT_EQ(startup_filter.filter(100.0F), 100.0F);

  SocSpikeFilter near_full_filter;
  EXPECT_FLOAT_EQ(near_full_filter.filter(99.0F), 99.0F);
  EXPECT_FLOAT_EQ(near_full_filter.filter(100.0F), 100.0F);
}

}  // namespace
}  // namespace tws_battery_driver_ros2
