#include <gtest/gtest.h>
#include <chrono>

#include "arwain.hpp"
#include "exceptions.hpp"
#include "input_parser.hpp"

extern std::streambuf* original_cout_buffer;

namespace arwain
{
    arwain::ReturnCode test_pressure();
    arwain::ReturnCode test_imu(const std::string&, const int);
    arwain::ReturnCode test_ori(int);
    arwain::ReturnCode test_mag();
    arwain::ReturnCode test_lora_tx();
    arwain::ReturnCode test_lora_rx();
    arwain::ReturnCode test_inference();
    arwain::ReturnCode interactive_test();
}

TEST(ArwainUtils, RollingAverage_default_constructor)
{
    RollingAverage rl;
    EXPECT_EQ(rl.current_average, 0);
    EXPECT_TRUE(rl.ready());
}

TEST(ArwainUtils, RollingAverage_overflow)
{
    RollingAverage roll{10};
    for (int i = 0; i < 20; i++)
    {
        EXPECT_NO_THROW(roll.feed(1));
    }
}

TEST(ArwainUtils, clamp_value)
{
    int value = -5;
    EXPECT_EQ(0, clamp_value(value, 0, 17));
    EXPECT_EQ(-5, clamp_value(value, -17, 11));
    EXPECT_EQ(-10, clamp_value(value, -17, -10));
}

TEST(Arwain, OperatingModeStreamOperator)
{
    // std::cout is globally turned off so turn it on for this test, then turn it off again at the end.
    std::cout.rdbuf(original_cout_buffer);

    testing::internal::CaptureStdout();
    std::cout << arwain::OperatingMode::AccelerometerCalibration;
    EXPECT_EQ("Accelerometer calibration", testing::internal::GetCapturedStdout());
    
    testing::internal::CaptureStdout();
    std::cout << arwain::OperatingMode::Inference;
    EXPECT_EQ("Inference", testing::internal::GetCapturedStdout());

    testing::internal::CaptureStdout();
    std::cout << arwain::OperatingMode::Idle;
    EXPECT_EQ("Idle/autocalibrating", testing::internal::GetCapturedStdout());

    testing::internal::CaptureStdout();
    std::cout << arwain::OperatingMode::Terminate;
    EXPECT_EQ("Terminate", testing::internal::GetCapturedStdout());

    testing::internal::CaptureStdout();
    std::cout << arwain::OperatingMode::DataCollection;
    EXPECT_EQ("Data collection", testing::internal::GetCapturedStdout());

    testing::internal::CaptureStdout();
    std::cout << arwain::OperatingMode::GyroscopeCalibration;
    EXPECT_EQ("Gyroscope calibration", testing::internal::GetCapturedStdout());

    testing::internal::CaptureStdout();
    std::cout << arwain::OperatingMode::MagnetometerCalibration;
    EXPECT_EQ("Magnetometer calibration", testing::internal::GetCapturedStdout());

    testing::internal::CaptureStdout();
    std::cout << arwain::OperatingMode::TestSerial;
    EXPECT_EQ("Test serial", testing::internal::GetCapturedStdout());

    testing::internal::CaptureStdout();
    std::cout << arwain::OperatingMode::TestStanceDetector;
    EXPECT_EQ("Test stance detector", testing::internal::GetCapturedStdout());

    testing::internal::CaptureStdout();
    std::cout << arwain::OperatingMode::SelfTest;
    EXPECT_EQ("Self test", testing::internal::GetCapturedStdout());

    testing::internal::CaptureStdout();
    std::cout << arwain::OperatingMode::InvalidMode;
    EXPECT_EQ("Mode not specified", testing::internal::GetCapturedStdout());

    std::cout.rdbuf(nullptr);
}

TEST(Arwain, ExceptionsNotImplemented)
{
    EXPECT_THROW(throw NotImplemented{"TEST"}, NotImplemented);
}

/** \brief Test that the sleep_ms utility function waits approximately the expected amount of time. */
TEST(ArwainUtils, sleep_ms)
{
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    sleep_ms(25);
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    
    int duration = (end - start).count();
    EXPECT_GT(duration, 25000000);
    EXPECT_LT(duration, 26000000);
}

/** \brief I don't know how this can be tested because I can't find any way to mock the secondary function calls. */
TEST(NOTREADY_Arwain, Arwain_Main)
{
    FAIL();
}

TEST(ArwainExceptions, NotImplemeted)
{
    EXPECT_THROW(throw NotImplemented{__FUNCTION__}, NotImplemented);
}

TEST(HARDWARE_NOTREADY_Arwain, test_pressure)
{
    arwain::test_pressure();
    FAIL();
}

TEST(HARDWARE_NOTREADY_Arwain, test_imu)
{
    arwain::test_imu("/dev/null", 1);
    FAIL();
}


TEST(HARDWARE_NOTREADY_Arwain, test_lora_rx)
{
    arwain::test_lora_rx();
    FAIL();
}

TEST(HARDWARE_NOTREADY_Arwain, test_inference)
{
    arwain::test_inference();
    FAIL();
}

TEST(HARDWARE_NOTREADY_Arwain, interactive_test)
{
    arwain::interactive_test();
    FAIL();
}

TEST(HARDWARE_NOTREADY_Arwain, test_mag)
{
    arwain::test_mag();
    FAIL();
}

TEST(HARDWARE_NOTREADY_Arwain, test_lora_tx)
{
    arwain::test_lora_tx();
    FAIL();
}

TEST(HARDWARE_NOTREADY_Arwain, test_ori)
{
    arwain::test_ori(1);
    FAIL();
}

double unwrap_phase_radians(double new_angle, double previous_angle);
TEST(ArwainUtils, unwrap_phase_radians)
{
    double angle = 0;
    double pi = 3.14159;
    
    EXPECT_EQ(1, unwrap_phase_radians(angle + 1, angle));
    EXPECT_EQ(2, unwrap_phase_radians(angle + 2, angle));
    EXPECT_EQ(3, unwrap_phase_radians(angle + 3, angle));
    EXPECT_EQ(4 - 2 * pi, unwrap_phase_radians(angle + 4, angle));

    EXPECT_EQ(-1, unwrap_phase_radians(angle - 1, angle));
    EXPECT_EQ(-2, unwrap_phase_radians(angle - 2, angle));
    EXPECT_EQ(-3, unwrap_phase_radians(angle - 3, angle));
    EXPECT_EQ(-4 + 2 * pi, unwrap_phase_radians(angle - 4, angle));
}

double unwrap_phase_degrees(double new_angle, double previous_angle);
TEST(ArwainUtils, unwrap_phase_degrees)
{
    double angle = 0;

    EXPECT_EQ(179, unwrap_phase_degrees(angle + 179, angle));
    EXPECT_EQ(181 - 360, unwrap_phase_degrees(angle + 181, angle));

    EXPECT_EQ(-179, unwrap_phase_degrees(angle - 179, angle));
    EXPECT_EQ(-181 + 360, unwrap_phase_degrees(angle - 181, angle));
}

std::string date_time_string();
TEST(NOTREADY_ArwainUtils, date_time_string)
{
    EXPECT_NO_THROW(date_time_string());
}

TEST(NOTREADY_ArwainUtils, compute_euler)
{
    Quaternion q{1, 0, 0, 0};
    EXPECT_NO_THROW(arwain::compute_euler(q));
}

TEST(NOTREADY_ArwainUtils, apply_quat_rotor_to_vector3)
{
    Vector3 v{1, 0, 0};
    Quaternion q{1, 0, 0, 0};
    EXPECT_NO_THROW(arwain::apply_quat_rotor_to_vector3(v, q));
}

TEST(Arwain, setup_log_folder_name_suffix_with_no_name)
{
    int j = 3;
    std::string program = "arwain_test";
    std::string command = "hello";
    std::string parameter = "1";
    char* input_array[3] = {program.data(), command.data(), parameter.data()};
    InputParser parser(j, input_array);

    arwain::setup_log_folder_name_suffix(parser);
    std::string empty = "";

    EXPECT_EQ(empty, arwain::folder_date_string_suffix);
}

TEST(Arwain, setup_log_folder_name_suffix_with_name)
{
    int j = 3;
    std::string program = "arwain_test";
    std::string command = "-name";
    std::string parameter = "example";
    char* input_array[3] = {program.data(),command.data(), parameter.data()};
    InputParser parser(j, input_array);

    arwain::setup_log_folder_name_suffix(parser);
    std::string name = "example";
    EXPECT_EQ(name, arwain::folder_date_string_suffix);
}

TEST(Arwain, Execute_Inference)
{
    FAIL();
}

TEST(Arwain, Rerun_Orientation_Filter)
{
    FAIL();
}

TEST(Arwain, Rerun_Floor_Tracker)
{
    FAIL();
}

TEST(Arwain, Date_Time_String)
{
    FAIL();
}

TEST(Arwain, Setup_Log_Directory)
{
    FAIL();
}

TEST(Arwain, deduce_calib_params)
{
    FAIL();
}
