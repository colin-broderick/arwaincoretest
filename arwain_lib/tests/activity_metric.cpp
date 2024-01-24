#include <gtest/gtest.h>

#include "arwain/activity_metric.hpp"

#include <arwain/vector3.hpp>

#define STUB

STUB RollingAverage::RollingAverage(unsigned int) {}
STUB double RollingAverage::get_value() { return 0; }
STUB void RollingAverage::feed(double) { }
STUB double Vector3::magnitude() const { return 0; }

TEST(ActivityMetric, feed_gyro)
{
    ActivityMetric act{10, 10};
    EXPECT_EQ(act.gyro_roller.get_value(), 0);
    act.feed_gyro(Vector3{0, 0, 0});
    EXPECT_EQ(act.gyro_roller.get_value(), 0);
    act.feed_gyro(Vector3{1, 1, 1});
    EXPECT_NE(act.gyro_roller.get_value(), 0);
}

TEST(ActivityMetric, feed_acce)
{
    ActivityMetric act{10, 10};
    EXPECT_EQ(act.acce_roller.get_value(), 0);
    act.feed_acce(Vector3{0, 0, 0});
    EXPECT_EQ(act.acce_roller.get_value(), 0);
    act.feed_acce(Vector3{1, 1, 1});
    EXPECT_NE(act.acce_roller.get_value(), 0);
}

TEST(ActivityMetric, feed_velocity)
{
    ActivityMetric act{10, 10};
    EXPECT_EQ(act.velo_roller.get_value(), 0);
    act.feed_velocity(Vector3{0, 0, 0});
    EXPECT_EQ(act.velo_roller.get_value(), 0);
    act.feed_velocity(Vector3{1, 1, 1});
    EXPECT_NE(act.velo_roller.get_value(), 0);
}

/** \brief The precise metric is not well defined at this time so this may break in future. */
TEST(ActivityMetric, read)
{
    ActivityMetric act{10, 10};
    act.feed_gyro(Vector3{1, 2, 3});
    act.feed_acce(Vector3{4, 5, 6});
    act.feed_velocity(Vector3{7, 8, 9});
    EXPECT_NE(act.read(), 0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
