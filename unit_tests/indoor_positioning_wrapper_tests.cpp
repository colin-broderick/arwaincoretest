#include <gtest/gtest.h>

#include "indoor_positioning_wrapper.hpp"
#include "floor_tracker.hpp"
#include "corner_detector.hpp"
#include "logger.hpp"
#include "vector3.hpp"
#include "arwain.hpp"

TEST(Indoor_Positioning, Update)
{
    FAIL();
}

TEST(Indoor_Positioning, getPosition)
{
    arwain::IndoorPositioningWrapper wrapper;
    Vector3 pos{1.0, 1.0, 1.0};

    EXPECT_EQ(pos, wrapper.getPosition());
}

TEST(Indoor_Positioning, getX)
{
    arwain::IndoorPositioningWrapper wrapper;
    double x = 1.0;
    EXPECT_EQ(x, wrapper.getX());
}

TEST(Indoor_Positioning, getY)
{
    arwain::IndoorPositioningWrapper wrapper;
    double y = 1.0;
    EXPECT_EQ(y, wrapper.getY());
}

TEST(Indoor_Positioning, getZ)
{
    arwain::IndoorPositioningWrapper wrapper;
    double z = 1.0;
    EXPECT_EQ(z, wrapper.getZ());
}
