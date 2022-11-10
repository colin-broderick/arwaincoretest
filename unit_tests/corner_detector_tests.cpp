#include <gtest/gtest.h>

#include "corner_detector.hpp"

TEST(CornerDetector, Constructor)
{

EXPECT_NO_THROW(arwain::CornerDetector(1,1.0,1.0));

}

TEST(CornerDetector, Update_Angle)
{
    Vector3 pos;
    pos.x = 1.0;
    pos.y = 1.0;
    pos.z = 1.0;

    arwain::CornerDetector detector(1, 1.0, 0.5);
    EXPECT_TRUE(detector.update(pos));
}

TEST(CornerDetector, Update_Insufficient_Motion)
{
    Vector3 pos;
    pos.x = 1.0;
    pos.y = 1.0;
    pos.z = 1.0;

    arwain::CornerDetector detector(1, 1.0, 2.0);
    EXPECT_FALSE(detector.update(pos));
}

TEST(CornerDetector, Update_Isufficient_Information)
{
    Vector3 pos;
    pos.x = 1.0;
    pos.y = 1.0;
    pos.z = 1.0;

    arwain::CornerDetector detector(3, 1.0, 0.5);
    EXPECT_FALSE(detector.update(pos));
}
