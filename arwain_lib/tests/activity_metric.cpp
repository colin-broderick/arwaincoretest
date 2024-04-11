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
    // TODO Skipped for now as there is nothing meaningful to test until we have a more mature design.
    GTEST_SKIP();
}

TEST(ActivityMetric, feed_acce)
{

// TODO Skipped for now as there is nothing meaningful to test until we have a more mature design.
    GTEST_SKIP();
}

TEST(ActivityMetric, feed_velocity)
{
    // TODO Skipped for now as there is nothing meaningful to test until we have a more mature design.
    GTEST_SKIP();
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
