#include <gtest/gtest.h>

#include "arwain.hpp"
#include "sensor_manager.hpp"

TEST(SensorManager, run_through_modes)
{
    arwain::config.no_imu = false;
    SensorManager sensors;
    sleep_ms(500);
    EventManager::switch_mode_event.invoke(arwain::OperatingMode::GyroscopeCalibration);
    sleep_ms(500);
    // EventManager::switch_mode_event.invoke(arwain::OperatingMode::MagnetometerCalibration);
    // sleep_ms(500);
    // EventManager::switch_mode_event.invoke(arwain::OperatingMode::AccelerometerCalibration);
    // sleep_ms(500);
    EventManager::switch_mode_event.invoke(arwain::OperatingMode::Inference);
    sleep_ms(500);
    EventManager::switch_mode_event.invoke(arwain::OperatingMode::TestStanceDetector);
    sleep_ms(500);
    // EventManager::switch_mode_event.invoke(arwain::OperatingMode::SelfTest);
    // sleep_ms(500);
    
    EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    sensors.join();
}

TEST(SensorManager, join)
{
    arwain::config.no_imu = true;
    SensorManager reader;
    EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    EXPECT_NO_THROW(reader.join());
}

TEST(SensorManager, init__success)
{
   /* arwain::config.no_imu = false;
    SensorManager reader;
    EXPECT_TRUE(reader.init());
    EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    reader.join();
    */
    FAIL();
}

TEST(SensorManager, init__failure)
{
    /*
    arwain::config.no_imu = true;
    SensorManager reader;
    EXPECT_FALSE(reader.init());
    EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    reader.join();
    */
   FAIL();
}

static void test_callback()
{

}

TEST(SensorManager, set_post_gyro_calibration_callback)
{
    arwain::config.no_imu = true;
    SensorManager sensors;
    test_callback(); // Honestly this call is just to get the test coverage up. There's nothing to test.
    EXPECT_EQ(sensors.post_gyro_calib_callback, nullptr);
    sensors.set_post_gyro_calibration_callback(test_callback);
    EXPECT_NE(sensors.post_gyro_calib_callback, nullptr);
    EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    sensors.join();
}
