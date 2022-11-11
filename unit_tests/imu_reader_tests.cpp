#include <gtest/gtest.h>

#include "imu_reader.hpp"

void shutdown_thread()
{
    std::this_thread::sleep_for(std::chrono::seconds{1});

    ImuProcessing::shutdown();
}

TEST(IMU_Reader, Set_Mode1)
{

    FAIL();
}

TEST(IMU_Reader, Set_Mode2)
{

    FAIL();
}

TEST(IMU_Reader, Shutdown)
{
    EXPECT_TRUE(ImuProcessing::shutdown());
}

TEST(IMU_Reader, Join)
{
    ImuProcessing::init();
    std::thread thread(shutdown_thread);
    thread.join();
    EXPECT_NO_THROW(ImuProcessing::join());
}

TEST(IMU_Reader, Init)
{

    FAIL();
}

TEST(IMU_Reader, Get_mode)
{
    arwain::OperatingMode mode;
    arwain::OperatingMode original_mode = arwain::OperatingMode::AutoCalibration;
    mode = ImuProcessing::get_mode();
    bool result = false;
    if (original_mode == mode)
    {
        result = true;
        EXPECT_TRUE(result);
    }
}
