#include <gtest/gtest.h>

#include "iim42652.hpp"
#include "vector3.hpp"

TEST(IIM42652, IIM42652)
{
    // The default constructor should always succeed since it doesn't try to access a device.
    EXPECT_NO_THROW(IIM42652<I2CDEVICEDRIVER>());

    // We expect this constructor to fail when passed junk address.
    EXPECT_THROW(IIM42652<I2CDEVICEDRIVER>(-1, "fail_bus"), std::runtime_error);
}

TEST(IIM42652, init__fail)
{
    // Cause init to fail by trying to provide an invalid I2C bus.
    EXPECT_THROW((IIM42652<I2CDEVICEDRIVER>{1, "fail_bus"}), std::runtime_error);
}

// TODO Incomplete, cannot test effectively without a list of ret codes and ways to generate errors
// Also hard to test with the mock interface.
TEST(IIM42652, imu_config)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    EXPECT_NO_THROW(imu.imu_config(0, 0));
}

TEST(IIM42652, set_resolutions)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.set_resolutions(1.5, 1.7);
    EXPECT_EQ(imu.accel_resolution, 1.5);
    EXPECT_EQ(imu.gyro_resolution, 1.7);
}

TEST(IIM42652, get_address)
{
    IIM42652<I2CDEVICEDRIVER> imu{1, "bus"};
    EXPECT_EQ(imu.get_address(), 1);
}

TEST(IIM42652, get_bus)
{
    IIM42652<I2CDEVICEDRIVER> imu{1, "bus"};
    EXPECT_EQ(imu.get_bus(), "bus");
}

TEST(IIM42652, update_gyro_bias)
{
    IIM42652<I2CDEVICEDRIVER> imu{1, "bus"};
    
    // Exceed the calib treshold and thereby reset the timer.
    imu.gyroscope_x = 1.1;
    imu.gyroscope_y = 1.1;
    imu.gyroscope_z = 1.1;
    imu.auto_calib_threshold = 1;

    EXPECT_FALSE(imu.update_gyro_bias());

    // Do not exceed the calib threshold but act before the timer is exceeded.
    imu.gyroscope_x = 0.1;
    imu.gyroscope_y = 0.1;
    imu.gyroscope_z = 0.1;
    EXPECT_FALSE(imu.update_gyro_bias());

    // Remain under threshold and exceed the timer, causing the calib data to change.
    imu.calib_time = 1;
    EXPECT_TRUE(imu.update_gyro_bias());
}

TEST(IIM42652, auto_calib_enabled)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.auto_calib_enabled_ = true;
    EXPECT_EQ(imu.auto_calib_enabled(), true);
    imu.auto_calib_enabled_ = false;
    EXPECT_EQ(imu.auto_calib_enabled(), false);
}

// Marked as not ready because can only be tested on hardware.
TEST(IIM42652, read_imu)
{
    IIM42652<I2CDEVICEDRIVER> driver(1, "test_driver");
    ImuData data;
    ImuData expected_data{0, 0, 0, 0, 0, 0};
    data = driver.read_imu();
    EXPECT_EQ(expected_data.acce, data.acce);
    EXPECT_EQ(expected_data.gyro, data.gyro);
}

TEST(IIM42652, set_gyro_bias)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.set_gyro_bias(1.5, 1.6, 1.7);
    EXPECT_EQ(imu.gyro_bias_x, 1.5);
    EXPECT_EQ(imu.gyro_bias_y, 1.6);
    EXPECT_EQ(imu.gyro_bias_z, 1.7);
}

TEST(IIM42652, set_accel_bias)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.set_accel_bias(1.5, 1.6, 1.7);
    EXPECT_EQ(imu.accel_bias_x, 1.5);
    EXPECT_EQ(imu.accel_bias_y, 1.6);
    EXPECT_EQ(imu.accel_bias_z, 1.7);
}

TEST(IIM42652, set_accel_scale)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.set_accel_scale(1.5, 1.6, 1.7);
    EXPECT_EQ(imu.accel_scale_x, 1.5);
    EXPECT_EQ(imu.accel_scale_y, 1.6);
    EXPECT_EQ(imu.accel_scale_z, 1.7);
}

TEST(IIM42652, set_correction_speed)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.set_correction_speed(-0.1);
    EXPECT_EQ(imu.correction_speed, 0);
    imu.set_correction_speed(0.1);
    EXPECT_EQ(imu.correction_speed, 0.1);
    imu.set_correction_speed(1.1);
    EXPECT_EQ(imu.correction_speed, 1);
}

TEST(IIM42652, enable_auto_calib)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.auto_calib_enabled_ = false;
    imu.enable_auto_calib();
    EXPECT_TRUE(imu.auto_calib_enabled_);
}

TEST(IIM42652, disable_auto_calib)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.auto_calib_enabled_ = true;
    imu.disable_auto_calib();
    EXPECT_FALSE(imu.auto_calib_enabled_);
}

TEST(IIM42652, get_gyro_calib)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.gyro_bias_x = 1.5;
    imu.gyro_bias_y = 1.5;
    imu.gyro_bias_z = 1.5;

    Vector3 bias{1.5, 1.5, 1.5};

    EXPECT_EQ(imu.get_gyro_calib(), bias);
}

TEST(IIM42652, get_gyro_calib_x)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.gyro_bias_x = 1.5;
    EXPECT_EQ(imu.get_gyro_calib_x(), 1.5);
}

TEST(IIM42652, get_gyro_calib_y)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.gyro_bias_y = 1.5;
    EXPECT_EQ(imu.get_gyro_calib_y(), 1.5);
}

TEST(IIM42652, get_gyro_calib_z)
{
    IIM42652<I2CDEVICEDRIVER> imu;
    imu.gyro_bias_z = 1.5;
    EXPECT_EQ(imu.get_gyro_calib_z(), 1.5);
}
