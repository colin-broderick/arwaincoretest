#include <gtest/gtest.h>

#include "imu_reader.hpp"

TEST(IMU_Reader, Join)
{
    ImuProcessing reader;
    arwain::system_mode = arwain::OperatingMode::Terminate;
    EXPECT_NO_THROW(reader.join());
}

TEST(IMU_Reader, Init_success)
{
   /* arwain::config.no_imu = false;
    ImuProcessing reader;
    EXPECT_TRUE(reader.init());
    arwain::system_mode = arwain::OperatingMode::Terminate;
    reader.join();
    */
    FAIL();
}

TEST(IMU_Reader, Init_failure)
{
    /*
    arwain::config.no_imu = true;
    ImuProcessing reader;
    EXPECT_FALSE(reader.init());
    arwain::system_mode = arwain::OperatingMode::Terminate;
    reader.join();
    */
   FAIL();
}

TEST(IMU_Reader, Set_Post_Gyro_Calibration_Callback)
{
   FAIL();
}
