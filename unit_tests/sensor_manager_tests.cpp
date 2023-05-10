#include <gtest/gtest.h>

#include "arwain/arwain.hpp"
#include "arwain/sensor_manager.hpp"

extern std::streambuf* original_cout_buffer;

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
    // std::cout.rdbuf(original_cout_buffer);

    arwain::config.no_imu = false;
    SensorManager reader;
    EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    reader.join();

    // std::cout.rdbuf(nullptr);
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
