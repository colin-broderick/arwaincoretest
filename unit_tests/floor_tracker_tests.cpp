#include <gtest/gtest.h>

#include "floor_tracker.hpp"

TEST(FloorTracker, Constructor)
{
    Vector3 pos;
    pos.x = 1;
    pos.y = 1;
    pos.z = 1;
    EXPECT_NO_THROW(arwain::FloorTracker(1,1.0,1.0));
    EXPECT_NO_THROW(arwain::FloorTracker(1,1.0,1.0, pos));
}

TEST(FloorTracker, Update_Seperation)
{
    Vector3 pos;
    pos.x = 1;
    pos.y = 1;
    pos.z = 1;
    arwain::FloorTracker tracker(1, 1.0, 2.0);
    EXPECT_FALSE(tracker.update(pos));
}

TEST(FloorTracker, Update_Window_Size)
{
    Vector3 pos;
    pos.x = 1;
    pos.y = 1;
    pos.z = 1;
    arwain::FloorTracker tracker(3, 1.0, 0.5);
    EXPECT_FALSE(tracker.update(pos));
}

TEST(FloorTracker, Update_Drift_Threshold)
{
    Vector3 pos;
    pos.x = 1;
    pos.y = 1;
    pos.z = 1;
    arwain::FloorTracker tracker(1, 1.0, 0.5);
    EXPECT_TRUE(tracker.update(pos));
}
