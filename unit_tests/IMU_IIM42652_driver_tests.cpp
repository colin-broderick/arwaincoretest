#include <gtest/gtest.h>

#include "IMU_IIM42652_driver.hpp"

TEST(Imu_driver, Constructor)
{
    EXPECT_NO_THROW(IMU_IIM42652());
    EXPECT_NO_THROW(IMU_IIM42652(1, "test_driver"));
}


TEST(Imu_driver, IMU_config)
{   //need a list of return codes and config options
    FAIL();
}

TEST(Imu_driver, set_resolutions)
{
    FAIL();//no getters for values set
}

TEST(Imu_driver, read_IMU)
{
    IMU_IIM42652 driver(1, "test_driver");
    ImuData data;
    ImuData expected_data{0,0,0,0,0,0};
    data = driver.read_IMU();
    EXPECT_EQ(expected_data.acce, data.acce);
    EXPECT_EQ(expected_data.gyro, data.gyro);
}

TEST(Imu_driver, calibrate_gyroscope)
{
    FAIL();
}

TEST(Imu_driver, calibration_accel_sample)
{
    FAIL();
}

TEST(Imu_driver, set_gyro_bias)
{
    FAIL();
}

TEST(Imu_driver, set_accel_bias)
{
    FAIL();
}

TEST(Imu_driver, set_accel_scale)
{
    FAIL();
}

TEST(Imu_driver, set_correction_speed)
{
    FAIL();
}

TEST(Imu_driver, enable_auto_calib)
{
    FAIL();
}

TEST(Imu_driver, enable_auto_calib1)
{
    FAIL();
}

TEST(Imu_driver, disable_auto_calib)
{
    FAIL();
}

TEST(Imu_driver, get_gyro_calib)
{
    FAIL();
}

TEST(Imu_driver, get_gyro_calib_x)
{
    FAIL();
}

TEST(Imu_driver, get_gyro_calib_y)
{
    FAIL();
}

TEST(Imu_driver, get_gyro_calib_z)
{
    FAIL();
}











