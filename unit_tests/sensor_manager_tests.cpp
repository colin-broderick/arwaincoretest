#include <gtest/gtest.h>

#include "sensor_manager.hpp"

TEST(SensorManager, run_through_modes)
{
    arwain::config.no_imu = false;
    SensorManager sensors;
    sleep_ms(500);
    arwain::system_mode = arwain::OperatingMode::GyroscopeCalibration;
    sleep_ms(500);
    // arwain::system_mode = arwain::OperatingMode::MagnetometerCalibration;
    // sleep_ms(500);
    // arwain::system_mode = arwain::OperatingMode::AccelerometerCalibration;
    // sleep_ms(500);
    arwain::system_mode = arwain::OperatingMode::Inference;
    sleep_ms(500);
    arwain::system_mode = arwain::OperatingMode::TestStanceDetector;
    sleep_ms(500);
    // arwain::system_mode = arwain::OperatingMode::SelfTest;
    // sleep_ms(500);
    
    arwain::system_mode = arwain::OperatingMode::Terminate;
    sensors.join();
}

TEST(SensorManager, Join)
{
    arwain::config.no_imu = true;
    SensorManager reader;
    arwain::system_mode = arwain::OperatingMode::Terminate;
    EXPECT_NO_THROW(reader.join());
}

TEST(HARDWARE_NOTREADY_SensorManager, Init_success)
{
   /* arwain::config.no_imu = false;
    SensorManager reader;
    EXPECT_TRUE(reader.init());
    arwain::system_mode = arwain::OperatingMode::Terminate;
    reader.join();
    */
    FAIL();
}

TEST(HARDWARE_NOTREADY_SensorManager, Init_failure)
{
    /*
    arwain::config.no_imu = true;
    SensorManager reader;
    EXPECT_FALSE(reader.init());
    arwain::system_mode = arwain::OperatingMode::Terminate;
    reader.join();
    */
   FAIL();
}

static void test_callback()
{

}

TEST(SensorManager, Set_Post_Gyro_Calibration_Callback)
{
    arwain::config.no_imu = true;
    SensorManager sensors;
    EXPECT_FALSE(sensors.init());
    EXPECT_EQ(sensors.post_gyro_calib_callback, nullptr);
    sensors.set_post_gyro_calibration_callback(test_callback);
    EXPECT_NE(sensors.post_gyro_calib_callback, nullptr);
    sensors.join();
}
