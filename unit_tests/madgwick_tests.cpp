//includes
#include <cmath>

#include <gtest/gtest.h>

#include "madgwick.hpp"

TEST(arwain__Madgwick, Madgwick)
{
    arwain::Madgwick mad;
    
    EXPECT_TRUE(mad.get_beta() == arwain::Madgwick::beta_default);
    EXPECT_EQ(mad.get_w(), 1);
    EXPECT_EQ(mad.get_x(), 0);
    EXPECT_EQ(mad.get_y(), 0);
    EXPECT_EQ(mad.get_z(), 0);
    EXPECT_NEAR(mad.get_frequency(), arwain::Madgwick::sample_frequency_default, 0.00001);
    EXPECT_NEAR(mad.get_inverse_sample_frequency(), 1.0 / arwain::Madgwick::sample_frequency_default, 0.00001);
    EXPECT_FALSE(mad.angles_updated());
}

TEST(arwain__Madgwick, update__mag_zeroes)
{
    arwain::Madgwick madgwick{2.0, 1.0};

    double pre_w = madgwick.get_w();
    double pre_x = madgwick.get_x();
    double pre_y = madgwick.get_y();
    double pre_z = madgwick.get_z();
    double pre_beta = madgwick.get_beta();
    double pre_freq = madgwick.get_frequency();
    
    madgwick.update(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
    
    double post_w = madgwick.get_w();
    double post_x = madgwick.get_x();
    double post_y = madgwick.get_y();
    double post_z = madgwick.get_z();
    double post_beta = madgwick.get_beta();
    double post_freq = madgwick.get_frequency();

    EXPECT_NE(pre_w, post_w);
    EXPECT_NE(pre_x, post_x);
    EXPECT_NE(pre_y, post_y);
    EXPECT_NE(pre_z, post_z);
    EXPECT_EQ(pre_beta, post_beta);
    EXPECT_EQ(pre_freq, post_freq);    

}

TEST(arwain__Madgwick, Madgwick__2)
{
    arwain::Madgwick mad(2.0, 1.0);
    
    EXPECT_EQ(mad.get_beta(), 1.0);
    EXPECT_EQ(mad.get_w(), 1);
    EXPECT_EQ(mad.get_x(), 0);
    EXPECT_EQ(mad.get_y(), 0);
    EXPECT_EQ(mad.get_z(), 0);
    EXPECT_NEAR(mad.get_frequency(), 2.0, 0.00001);
    EXPECT_NEAR(mad.get_inverse_sample_frequency(), 1.0 / 2.0, 0.00001);
    EXPECT_FALSE(mad.angles_updated());
}

TEST(arwain__Madgwick, get_beta)
{
    arwain::Madgwick mad;
    double beta = mad.get_beta();
    EXPECT_TRUE(beta == 0.1);
}

TEST(arwain__Madgwick, get_w)
{
    arwain::Madgwick mad;
    double w = mad.get_w();
    EXPECT_TRUE(w == 1.0);
}

TEST(arwain__Madgwick, get_x)
{
    arwain::Madgwick mad;
    double x = mad.get_x();
    EXPECT_TRUE(x == 0.0);
}

TEST(arwain__Madgwick, get_y)
{
    arwain::Madgwick mad;
    double y = mad.get_y();
    EXPECT_TRUE(y == 0.0);
}

TEST(arwain__Madgwick, get_z)
{
    arwain::Madgwick mad;
    double z = mad.get_z();
    EXPECT_TRUE(z == 0.0);
}

TEST(arwain__Madgwick, get_roll)
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

TEST(arwain__Madgwick, get_pitch)
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

TEST(arwain__Madgwick, get_yaw)
{
    /* Yaw is a bit tricky to test as it's one of the biggest theoretical problems faced by ARWAIN.
    It responds 'unpredictably' to sensor data and therefore we do only simple tests on its getter.
    */
    arwain::Madgwick mad;

    // Yaw should initially be zero.
    EXPECT_EQ(mad.get_yaw(), 0.0);

    // After updates, it will be different to zero, and will probably never quite be zero again.
    mad.update(0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
    EXPECT_NE(mad.get_yaw(), 0.0);
}

TEST(arwain__Madgwick, get_roll_radians)
{
    arwain::Madgwick mad;

    // Roll should be initially be zero.
    EXPECT_EQ(mad.get_roll_radians(), 0);

    // After some noisy data updates, roll should be non-zero.
    mad.update(0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
    EXPECT_NE(mad.get_roll_radians(), 0);

    // After many gravity-aligned updates, roll should be close to zero;
    // we check it is less than 0.02 radians.
    for (int i = 0; i < 50000; i++)
    {
        mad.update(0, 0, 0, 0, 0, 10);
    }
    EXPECT_LT(mad.get_roll_radians(), 0.02);

    // After many anti-gravity-aligned updates, roll should be close to pi radians.
    for (int i = 0; i < 50000; i++)
    {
        mad.update(0, 0, 0, 0, 0, -10);
    }
    EXPECT_GT(std::abs(mad.get_roll_radians()), (1.0 - 1.0/180.0) * 3.141593);
    EXPECT_LT(std::abs(mad.get_roll_radians()), 3.141593);
}

TEST(arwain__Madgwick, get_pitch_radians)
{
    arwain::Madgwick mad;

    // Pitch should be initially be zero.
    EXPECT_EQ(mad.get_pitch_radians(), 0);

    // After some noisy data updates, pitch should be non-zero.
    mad.update(0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
    EXPECT_NE(mad.get_pitch_radians(), 0);

    // After many gravity-aligned updates, pitch should be close to zero;
    // we check it is less than 0.02 radians.
    for (int i = 0; i < 50000; i++)
    {
        mad.update(0, 0, 0, 0, 0, 10);
    }
    EXPECT_LT(mad.get_pitch_radians(), 0.02);

    // After many anti-gravity-aligned updates, pitch should be close to 0 radians.
    for (int i = 0; i < 50000; i++)
    {
        mad.update(0, 0, 0, 0, 0, -10);
    }
    EXPECT_GT(std::abs(mad.get_pitch_radians()), 0.0);
    EXPECT_LT(std::abs(mad.get_pitch_radians()), 1.0 / 180.0 * 3.141593);
}

TEST(arwain__Madgwick, get_yaw_radians)
{
    /* Yaw is a bit tricky to test as it's one of the biggest theoretical problems faced by ARWAIN.
    It responds 'unpredictably' to sensor data and therefore we do only simple tests on its getter.
    */
    arwain::Madgwick mad;

    // Yaw should initially be zero.
    EXPECT_EQ(mad.get_yaw_radians(), 0.0);

    // After updates, it will be different to zero, and will probably never quite be zero again.
    mad.update(0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
    EXPECT_NE(mad.get_yaw_radians(), 0.0);
}

TEST(arwain__Madgwick, set_q)
{
    arwain::Madgwick mad;
    mad.set_q(1.0, 2.0, 3.0, 4.0);
    EXPECT_TRUE(mad.get_w() == 1.0);
    EXPECT_TRUE(mad.get_x() == 2.0);
    EXPECT_TRUE(mad.get_y() == 3.0);
    EXPECT_TRUE(mad.get_z() == 4.0);
}

TEST(arwain__Madgwick, set_beta)
{
    arwain::Madgwick mad;
    mad.set_beta(1.0);
    EXPECT_TRUE(mad.get_beta() == 1.0);
}

TEST(arwain__Madgwick, update__6_args)
{
    /* The state that a Madgwick filter should move to after an update is quite a complex question, and in my 
    opinion not something it is realistic to duplicate in a small unit test. What we can say is that after an update,
    certain properties should definitely be different and certain properties should definitely no unchanged. More
    detailed testing is the domain of functional/system tests. */

    // Testing void update(double gx, double gy, double gz, double ax, double ay, double az);

    arwain::Madgwick mad;

    double pre_w = mad.get_w();
    double pre_x = mad.get_x();
    double pre_y = mad.get_y();
    double pre_z = mad.get_z();
    double pre_beta = mad.get_beta();
    double pre_freq = mad.get_frequency();
    
    mad.update(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    
    double post_w = mad.get_w();
    double post_x = mad.get_x();
    double post_y = mad.get_y();
    double post_z = mad.get_z();
    double post_beta = mad.get_beta();
    double post_freq = mad.get_frequency();

    EXPECT_NE(pre_w, post_w);
    EXPECT_NE(pre_x, post_x);
    EXPECT_NE(pre_y, post_y);
    EXPECT_NE(pre_z, post_z);
    EXPECT_EQ(pre_beta, post_beta);
    EXPECT_EQ(pre_freq, post_freq);    
}

TEST(arwain__Madgwick, update__7_args)
{
    // Testing void update(double timestamp, double gx, double gy, double gz, double ax, double ay, double az);
    
    arwain::Madgwick mad;

    double pre_w = mad.get_w();
    double pre_x = mad.get_x();
    double pre_y = mad.get_y();
    double pre_z = mad.get_z();
    double pre_beta = mad.get_beta();
    double pre_freq = mad.get_frequency();
    
    mad.update(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    
    double post_w = mad.get_w();
    double post_x = mad.get_x();
    double post_y = mad.get_y();
    double post_z = mad.get_z();
    double post_beta = mad.get_beta();
    double post_freq = mad.get_frequency();

    EXPECT_NE(pre_w, post_w);
    EXPECT_NE(pre_x, post_x);
    EXPECT_NE(pre_y, post_y);
    EXPECT_NE(pre_z, post_z);
    EXPECT_EQ(pre_beta, post_beta);
    EXPECT_EQ(pre_freq, post_freq);
}

TEST(arwain__Madgwick, update__9_args)
{
    // Testing void update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
    
    arwain::Madgwick mad;

    double pre_w = mad.get_w();
    double pre_x = mad.get_x();
    double pre_y = mad.get_y();
    double pre_z = mad.get_z();
    double pre_beta = mad.get_beta();
    double pre_freq = mad.get_frequency();
    
    mad.update(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    
    double post_w = mad.get_w();
    double post_x = mad.get_x();
    double post_y = mad.get_y();
    double post_z = mad.get_z();
    double post_beta = mad.get_beta();
    double post_freq = mad.get_frequency();

    EXPECT_NE(pre_w, post_w);
    EXPECT_NE(pre_x, post_x);
    EXPECT_NE(pre_y, post_y);
    EXPECT_NE(pre_z, post_z);
    EXPECT_EQ(pre_beta, post_beta);
    EXPECT_EQ(pre_freq, post_freq);
}

TEST(arwain__Madgwick, update__10_args)
{
    // Testing void update(double timestamp, double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
    
    arwain::Madgwick mad;

    double pre_w = mad.get_w();
    double pre_x = mad.get_x();
    double pre_y = mad.get_y();
    double pre_z = mad.get_z();
    double pre_beta = mad.get_beta();
    double pre_freq = mad.get_frequency();
    
    mad.update(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
    
    double post_w = mad.get_w();
    double post_x = mad.get_x();
    double post_y = mad.get_y();
    double post_z = mad.get_z();
    double post_beta = mad.get_beta();
    double post_freq = mad.get_frequency();

    EXPECT_NE(pre_w, post_w);
    EXPECT_NE(pre_x, post_x);
    EXPECT_NE(pre_y, post_y);
    EXPECT_NE(pre_z, post_z);
    EXPECT_EQ(pre_beta, post_beta);
    EXPECT_EQ(pre_freq, post_freq);
}

TEST(arwain__Madgwick, get_euler_angles_degrees)
{
    arwain::Madgwick f;
    f.get_euler_angles_degrees(1, 0, 0, 0);
    f.get_euler_angles_radians(1, 0, 0, 0);
}
