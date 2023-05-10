#include <gtest/gtest.h>

#include "arwain/floor_tracker.hpp"

TEST(arwain__FloorTracker, FloorTracker)
{
    Vector3 pos;
    pos.x = 1;
    pos.y = 1;
    pos.z = 1;
    EXPECT_NO_THROW(arwain::FloorTracker(1,1.0,1.0));
    EXPECT_NO_THROW(arwain::FloorTracker(1,1.0,1.0, pos));
}

TEST(arwain__FloorTracker, update__separation)
{
    Vector3 pos;
    pos.x = 1;
    pos.y = 1;
    pos.z = 1;
    arwain::FloorTracker tracker(1, 1.0, 2.0);
    EXPECT_FALSE(tracker.update(pos));
}

TEST(arwain__FloorTracker, udpate__window_size)
{
    Vector3 pos;
    pos.x = 1;
    pos.y = 1;
    pos.z = 1;
    arwain::FloorTracker tracker(3, 1.0, 0.5);
    EXPECT_FALSE(tracker.update(pos));
}

TEST(arwain__FloorTracker, update__drift_threshold)
{
    Vector3 pos;
    pos.x = 1;
    pos.y = 1;
    pos.z = 1;
    arwain::FloorTracker tracker(1, 1.0, 0.5);
    EXPECT_TRUE(tracker.update(pos));
}
