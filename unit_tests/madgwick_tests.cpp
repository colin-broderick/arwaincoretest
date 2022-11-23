//includes
#include <cmath>

#include "madgwick.hpp"
#include "gtest/gtest.h"

TEST(Madgwick, Default_Constructor)
{
    arwain::Madgwick mad;
    
    EXPECT_TRUE(mad.get_beta() == 0.1);
    //all other parts of the constructor can't be verified due to being private with no getters.
}

TEST(Madgwick, Constructor)
{
    arwain::Madgwick mad(2.0, 1.0);
    
    EXPECT_TRUE(mad.get_beta() == 1.0);
    //sample frequency can't be verified - NEEDS TO BE VERIFIED!!!
    //all other parts of the constructor can't be verified due to being private with no getters.
    FAIL();
}

TEST(Madgwick, Get_Beta)
{
    arwain::Madgwick mad;
    double beta = mad.get_beta();
    EXPECT_TRUE(beta == 0.1);
}

TEST(Madgwick, Get_W)
{
    arwain::Madgwick mad;
    double w = mad.get_w();
    EXPECT_TRUE(w == 1.0);
}

TEST(Madgwick, Get_X)
{
    arwain::Madgwick mad;
    double x = mad.get_x();
    EXPECT_TRUE(x == 0.0);
}

TEST(Madgwick, Get_Y)
{
    arwain::Madgwick mad;
    double y = mad.get_y();
    EXPECT_TRUE(y == 0.0);
}

TEST(Madgwick, Get_Z)
{
    arwain::Madgwick mad;
    double z = mad.get_z();
    EXPECT_TRUE(z == 0.0);
}

TEST(Madgwick, Get_roll)
{
    arwain::Madgwick mad;

    // Roll should be initially be zero.
    EXPECT_EQ(mad.get_roll(), 0);

    // After some noisy data updates, roll should be non-zero.
    mad.update(0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
    EXPECT_NE(mad.get_roll(), 0);

    // After many gravity-aligned updates, roll should be close to zero;
    // we check it is less than 1 degree.
    for (int i = 0; i < 50000; i++)
    {
        mad.update(0, 0, 0, 0, 0, 10);
    }
    EXPECT_LT(mad.get_roll(), 1.0);

    // After many anti-gravity-aligned updates, roll should be close to 180 degrees.
    for (int i = 0; i < 50000; i++)
    {
        mad.update(0, 0, 0, 0, 0, -10);
    }
    EXPECT_GT(std::abs(mad.get_roll()), 179.0);
    EXPECT_LT(std::abs(mad.get_roll()), 180.0);
}

TEST(Madgwick, Get_pitch)
{
    arwain::Madgwick mad;

    // Pitch should be initially be zero.
    EXPECT_EQ(mad.get_pitch(), 0);

    // After some noisy data updates, pitch should be non-zero.
    mad.update(0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
    EXPECT_NE(mad.get_pitch(), 0);

    // After many gravity-aligned updates, pitch should be close to zero;
    // we check it is less than 1 degree.
    for (int i = 0; i < 50000; i++)
    {
        mad.update(0, 0, 0, 0, 0, 10);
    }
    EXPECT_LT(mad.get_pitch(), 1.0);

    // After many anti-gravity-aligned updates, pitch should be close to 0 degrees.
    for (int i = 0; i < 50000; i++)
    {
        mad.update(0, 0, 0, 0, 0, -10);
    }
    EXPECT_GT(std::abs(mad.get_pitch()), 0.0);
    EXPECT_LT(std::abs(mad.get_pitch()), 1.0);
}

TEST(Madgwick, Get_yaw)
{
    arwain::Madgwick mad;
    double pitch = mad.get_yaw();
    //EXPECT_TRUE(yaw == 0.0);
    FAIL();
}

TEST(Madgwick, Get_roll_radians)
{
    arwain::Madgwick mad;
    double roll_radians = mad.get_roll_radians();
    //EXPECT_TRUE(roll_radians == 0.0);
    FAIL();
}

TEST(Madgwick, Get_pitch_radians)
{
    arwain::Madgwick mad;
    double pitch_radians = mad.get_pitch_radians();
    //EXPECT_TRUE(pitch_radians == 0.0);
    FAIL();
}

TEST(Madgwick, Get_pitch_yaw)
{
    arwain::Madgwick mad;
    double yaw_radians = mad.get_yaw_radians();
    //EXPECT_TRUE(yaw_radians == 0.0);
    FAIL();
}

TEST(Madgwick, set_q)
{
    arwain::Madgwick mad;
    mad.set_q(1.0, 2.0, 3.0, 4.0);
    EXPECT_TRUE(mad.get_w() == 1.0);
    EXPECT_TRUE(mad.get_x() == 2.0);
    EXPECT_TRUE(mad.get_y() == 3.0);
    EXPECT_TRUE(mad.get_z() == 4.0);
}

TEST(Madgwick, set_beta)
{
    arwain::Madgwick mad;
    mad.set_beta(1.0);
    EXPECT_TRUE(mad.get_beta() == 1.0);
}

TEST(Madgwick, update)
{
    arwain::Madgwick mad;
    mad.update(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    //mad.set_beta(1.0);
    //EXPECT_TRUE(mad.get_beta() == 1.0);
    FAIL();
}
