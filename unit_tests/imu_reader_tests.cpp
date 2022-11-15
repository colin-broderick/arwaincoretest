#include <gtest/gtest.h>

#include "imu_reader.hpp"

void shutdown_thread()
{
    std::this_thread::sleep_for(std::chrono::seconds{1});

    ImuProcessing::shutdown();
}

TEST(IMU_Reader, Set_Mode_Terminate)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::Terminate);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_Inference)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::Inference);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_Auto_Calabration)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::AutoCalibration);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_Self_Test)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::SelfTest);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_GyroscopeCalibration)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::GyroscopeCalibration);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_MagnetometerCalibration)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::MagnetometerCalibration);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_AccelerometerCalibration)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::AccelerometerCalibration);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_TestSerial)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::TestSerial);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_TestStanceDetector)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::TestStanceDetector);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_DataCollection)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::DataCollection);
    EXPECT_TRUE(std::get<0>(result));
}

void example_callback()
{
    // do something
}

TEST(IMU_Reader, Set_Mode_With_Func_Terminate)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::Terminate, example_callback);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_With_Func_Inference)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::Inference, example_callback);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_With_Func_AutoCalibration)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::AutoCalibration, example_callback);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_With_Func_SelfTest)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::SelfTest, example_callback);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_With_Func_GyroscopeCalibration)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::GyroscopeCalibration, example_callback);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_With_Func_MagnetometerCalibration)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::MagnetometerCalibration, example_callback);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_With_Func_AccelerometerCalibration)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::AccelerometerCalibration, example_callback);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_With_Func_TestSerial)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::TestSerial, example_callback);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_With_Func_TestStanceDetector)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::TestStanceDetector, example_callback);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(IMU_Reader, Set_Mode_With_Func_DataCollection)
{
    std::tuple<bool, std::string> result;
    result = ImuProcessing::set_mode(arwain::OperatingMode::DataCollection, example_callback);
    EXPECT_TRUE(std::get<0>(result));
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

TEST(IMU_Reader, Init_success)
{
    arwain::config.no_imu = false;
    EXPECT_TRUE(ImuProcessing::init());
    ImuProcessing::shutdown();
    ImuProcessing::join();
}

TEST(IMU_Reader, Init_failure)
{
    arwain::config.no_imu = true;
    EXPECT_FALSE(ImuProcessing::init());
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
