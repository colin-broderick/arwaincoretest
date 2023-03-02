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

TEST(Arwain, test_pressure)
{
    EXPECT_EQ(arwain::ReturnCode::Success, arwain::test_pressure());
}

TEST(Arwain, test_imu)
{
    EXPECT_EQ(arwain::ReturnCode::Success, arwain::test_imu("/dev/null", 1));
}


TEST(HARDWARE_NOTREADY_Arwain, test_lora_rx)
{
    arwain::test_lora_rx();
    FAIL();
}

TEST(Arwain, test_inference)
{
    FAIL();
    // Gets stuck in a loop somewhere and never returns, even with MockInferrer.
    EXPECT_EQ(arwain::ReturnCode::Success, arwain::test_inference());
}

// TEST(HARDWARE_NOTREADY_Arwain, interactive_test)
// {
//     arwain::interactive_test();
//     FAIL();
// }

TEST(Arwain, test_mag)
{
    // TODO Incomplete coverage because chip ID returned by func is not 0x3D.
    EXPECT_EQ(arwain::ReturnCode::FailedMagnetometer, arwain::test_mag());
}

TEST(ArwainUtils, setup_log_directory)
{
    arwain::config.config_file = "/etc/randomnonsensefile";
    // Will throw exception if the expected config location is not current.
    EXPECT_THROW(arwain::setup_log_directory(), std::exception);

    // Get into the branch which nulls the filename.
    arwain::folder_date_string_suffix = "nullname";
    EXPECT_THROW(arwain::setup_log_directory(), std::exception);

    // Get int the branch which sets the folder data string suffix if not already set.
    arwain::folder_date_string_suffix = "randomjunk";
    EXPECT_THROW(arwain::setup_log_directory(), std::exception);

    // To get past the exception, provide a valid conf file to copy.
    std::ofstream tempfile{"./testfile.txt"};
    tempfile << "Test text\n";
    tempfile.close();
    arwain::config.config_file = "./testfile.txt";
    EXPECT_NO_THROW(arwain::setup_log_directory());

    // We sleep here to make sure the next call creates a new directory.
    sleep_ms(1000);

    // Having done all that, the error log should now be open and we can
    // hit the branch that closes it.
    EXPECT_NO_THROW(arwain::setup_log_directory());

    std::filesystem::remove("./tempfile.txt");
}

TEST(Arwain, arwain_execute)
{
    // Turn off all the options we can to prevent deep calls into
    // seconday functions.
    arwain::config.no_imu = true;
    arwain::config.no_inference = true;
    arwain::config.no_lora = true;
    arwain::config.no_pressure = true;
    arwain::config.no_cli = true;
    arwain::config.use_ips = false;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    EXPECT_EQ(arwain::execute_jobs(), arwain::ReturnCode::Success);
}

static void terminate_after_ms(int ms)
{
    static std::thread th = std::thread{
        [ms]()
        {
            sleep_ms(ms);
            arwain::system_mode = arwain::OperatingMode::Terminate;
        }
    };
}

TEST(ArwainUtils, calibrate_magnetometers)
{
    FAIL();
    terminate_after_ms(3500);
    EXPECT_EQ(arwain::calibrate_magnetometers(), arwain::ReturnCode::Success);
}

TEST(HARDWARE_NOTREADY_Arwain, test_lora_tx)
{
    arwain::test_lora_tx();
    FAIL();
}

TEST(Arwain, test_ori)
{
    EXPECT_EQ(arwain::ReturnCode::Success, arwain::test_ori(1));
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
