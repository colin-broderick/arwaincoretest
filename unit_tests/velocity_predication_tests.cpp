#include <gtest/gtest.h>

#include "velocity_prediction.hpp"

void velocity_shutdown_thread()
{
    std::this_thread::sleep_for(std::chrono::seconds{1});
    PositionVelocityInference::shutdown();
}

TEST(Velocity_Prediction, Set_Mode_Terminate)
{
    std::tuple<bool, std::string> result;
    result = PositionVelocityInference::set_mode(arwain::OperatingMode::Terminate);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(Velocity_Prediction, Set_Mode_Inference)
{
    std::tuple<bool, std::string> result;
    result = PositionVelocityInference::set_mode(arwain::OperatingMode::Inference);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(Velocity_Prediction, Set_Mode_Auto_Calabrition)
{
    std::tuple<bool, std::string> result;
    result = PositionVelocityInference::set_mode(arwain::OperatingMode::AutoCalibration);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(Velocity_Prediction, Set_Mode_Self_Test)
{
    std::tuple<bool, std::string> result;
    result = PositionVelocityInference::set_mode(arwain::OperatingMode::SelfTest);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(Velocity_Prediction, Set_Mode_GyroscopeCalibration)
{
    std::tuple<bool, std::string> result;
    result = PositionVelocityInference::set_mode(arwain::OperatingMode::GyroscopeCalibration);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(Velocity_Prediction, Set_Mode_MagnetometerCalibration)
{
    std::tuple<bool, std::string> result;
    result = PositionVelocityInference::set_mode(arwain::OperatingMode::MagnetometerCalibration);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(Velocity_Prediction, Set_Mode_AccelerometerCalibration)
{
    std::tuple<bool, std::string> result;
    result = PositionVelocityInference::set_mode(arwain::OperatingMode::AccelerometerCalibration);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(Velocity_Prediction, Set_Mode_TestSerial)
{
    std::tuple<bool, std::string> result;
    result = PositionVelocityInference::set_mode(arwain::OperatingMode::TestSerial);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(Velocity_Prediction, Set_Mode_TestStanceDetector)
{
    std::tuple<bool, std::string> result;
    result = PositionVelocityInference::set_mode(arwain::OperatingMode::TestStanceDetector);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(Velocity_Prediction, Set_Mode_DataCollection)
{
    std::tuple<bool, std::string> result;
    result = PositionVelocityInference::set_mode(arwain::OperatingMode::DataCollection);
    EXPECT_TRUE(std::get<0>(result));
}

TEST(Velocity_Prediction, Get_Mode)
{
    arwain::OperatingMode mode;
    arwain::OperatingMode original_mode = arwain::OperatingMode::AutoCalibration;
    mode = PositionVelocityInference::get_mode();
    bool result = false;
    if (original_mode == mode)
    {
        result = true;
        EXPECT_TRUE(result);
    }
}

TEST(Velocity_Prediction, Shutdown)
{
    EXPECT_TRUE(PositionVelocityInference::shutdown());
}

TEST(Velocity_Prediction, Join)
{
    PositionVelocityInference::init();
    std::thread thread(velocity_shutdown_thread);
    thread.join();
    EXPECT_NO_THROW(PositionVelocityInference::join());
}

TEST(Velocity_Prediction, Init_Success)
{
    arwain::config.no_inference = false;
    EXPECT_TRUE(PositionVelocityInference::init());
    PositionVelocityInference::shutdown();
    PositionVelocityInference::join();
}

TEST(Velocity_Prediction, Init_Failure)
{
    arwain::config.no_inference = true;
    EXPECT_FALSE(PositionVelocityInference::init());
}
