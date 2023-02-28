#include <gtest/gtest.h>

#include "iim42652.hpp"
#include "vector3.hpp"

TEST(Imu_driver, Constructor)
{
    // The default constructor should always succeed since it doesn't try to access a device.
    EXPECT_NO_THROW(IMU_IIM42652());

    // We expect this constructor to fail when passed junk address.
    EXPECT_THROW(IMU_IIM42652(1, "test_driver"), std::runtime_error);
}

// This marked as not ready since it can only be tested with hardware.
TEST(NOTREADY_Imu_driver, IMU_config)
{   //need a list of return codes and config options
    FAIL();
}

TEST(Imu_driver, set_resolutions)
{
    IMU_IIM42652 imu;
    imu.set_resolutions(1.5, 1.7);
    EXPECT_EQ(imu.accel_resolution, 1.5);
    EXPECT_EQ(imu.gyro_resolution, 1.7);
}

// Marked as not ready because can only be tested on hardware.
TEST(NOTREADY_Imu_driver, read_IMU)
{
    IMU_IIM42652 driver(1, "test_driver");
    ImuData data;
    ImuData expected_data{0,0,0,0,0,0};
    data = driver.read_IMU();
    EXPECT_EQ(expected_data.acce, data.acce);
    EXPECT_EQ(expected_data.gyro, data.gyro);
}

TEST(Imu_driver, set_gyro_bias)
{
    IMU_IIM42652 imu;
    imu.set_gyro_bias(1.5, 1.6, 1.7);
    EXPECT_EQ(imu.gyro_bias_x, 1.5);
    EXPECT_EQ(imu.gyro_bias_y, 1.6);
    EXPECT_EQ(imu.gyro_bias_z, 1.7);
}

TEST(Imu_driver, set_accel_bias)
{
     IMU_IIM42652 imu;
    imu.set_accel_scale(1.5, 1.6, 1.7);
    EXPECT_EQ(imu.accel_scale_x, 1.5);
    EXPECT_EQ(imu.accel_scale_y, 1.6);
    EXPECT_EQ(imu.accel_scale_z, 1.7);
}

TEST(Imu_driver, set_accel_scale)
{
    IMU_IIM42652 imu;
    imu.set_accel_scale(1.5, 1.6, 1.7);
    EXPECT_EQ(imu.accel_scale_x, 1.5);
    EXPECT_EQ(imu.accel_scale_y, 1.6);
    EXPECT_EQ(imu.accel_scale_z, 1.7);
}

TEST(Imu_driver, set_correction_speed)
{
    IMU_IIM42652 imu;
    imu.set_correction_speed(-0.1);
    EXPECT_EQ(imu.correction_speed, 0);
    imu.set_correction_speed(0.1);
    EXPECT_EQ(imu.correction_speed, 0.1);
    imu.set_correction_speed(1.1);
    EXPECT_EQ(imu.correction_speed, 1);
}

TEST(Imu_driver, enable_auto_calib)
{
    IMU_IIM42652 imu;
    imu.auto_calib_enabled_ = false;
    imu.enable_auto_calib();
    EXPECT_TRUE(imu.auto_calib_enabled_);
}

TEST(Imu_driver, disable_auto_calib)
{
    IMU_IIM42652 imu;
    imu.auto_calib_enabled_ = true;
    imu.disable_auto_calib();
    EXPECT_FALSE(imu.auto_calib_enabled_);
}

TEST(Imu_driver, get_gyro_calib)
{
    IMU_IIM42652 imu;
    imu.gyro_bias_x = 1.5;
    imu.gyro_bias_y = 1.5;
    imu.gyro_bias_z = 1.5;

    Vector3 bias{1.5, 1.5, 1.5};

    EXPECT_EQ(imu.get_gyro_calib(), bias);
}

TEST(Imu_driver, get_gyro_calib_x)
{
    IMU_IIM42652 imu;
    imu.gyro_bias_x = 1.5;
    EXPECT_EQ(imu.get_gyro_calib_x(), 1.5);
}

TEST(Imu_driver, get_gyro_calib_y)
{
    IMU_IIM42652 imu;
    imu.gyro_bias_y = 1.5;
    EXPECT_EQ(imu.get_gyro_calib_y(), 1.5);
}

TEST(Imu_driver, get_gyro_calib_z)
{
    IMU_IIM42652 imu;
    imu.gyro_bias_z = 1.5;
    EXPECT_EQ(imu.get_gyro_calib_z(), 1.5);
}
