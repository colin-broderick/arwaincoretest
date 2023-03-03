#include <gtest/gtest.h>

#include "corner_detector.hpp"

TEST(arwain__CornerDetector, CornerDetector)
{
    EXPECT_NO_THROW(arwain::CornerDetector(1,1.0,1.0));
}

TEST(arwain__CornerDetector, update)
{
    Vector3 pos;
    pos.x = 1.0;
    pos.y = 1.0;
    pos.z = 1.0;

    arwain::CornerDetector detector(1, 1.0, 0.5);
    EXPECT_TRUE(detector.update(pos));
}

TEST(arwain__CornerDetector, update__insufficient_motion)
{
    Vector3 pos;
    pos.x = 1.0;
    pos.y = 1.0;
    pos.z = 1.0;

    arwain::CornerDetector detector(1, 1.0, 2.0);
    EXPECT_FALSE(detector.update(pos));
}

TEST(arwain__CornerDetector, udpate__insufficient_information)
{
    Vector3 pos;
    pos.x = 1.0;
    pos.y = 1.0;
    pos.z = 1.0;

    arwain::CornerDetector detector(3, 1.0, 0.5);
    EXPECT_FALSE(detector.update(pos));
}
