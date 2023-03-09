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

TEST(RollingAverage, RollingAverage__default)
{
    RollingAverage rl;
    EXPECT_EQ(rl.current_average, 0);
    EXPECT_TRUE(rl.ready());
}

TEST(RollingAverage, feed__overflow)
{
    RollingAverage roll{10};
    for (int i = 0; i < 20; i++)
    {
        EXPECT_NO_THROW(roll.feed(1));
    }
}

TEST(FreeFuncs, clamp_value)
{
    int value = -5;
    EXPECT_EQ(0, clamp_value(value, 0, 17));
    EXPECT_EQ(-5, clamp_value(value, -17, 11));
    EXPECT_EQ(-10, clamp_value(value, -17, -10));
}

TEST(OperatingMode, ostream)
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

TEST(NotImplemented, throw)
{
    EXPECT_THROW(throw NotImplemented{"TEST"}, NotImplemented);
}

/** \brief Test that the sleep_ms utility function waits approximately the expected amount of time. */
TEST(FreeFuncs, sleep_ms)
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

TEST(arwain__FreeFuncs, test_pressure)
{
    EXPECT_EQ(arwain::ReturnCode::Success, arwain::test_pressure());
}

TEST(arwain__FreeFuncs, test_imu)
{
    EXPECT_EQ(arwain::ReturnCode::Success, arwain::test_imu("/dev/null", 1));
}

/** \brief Cannot test the main loop inside test_lora_rx, since that's waiting on hardware returning something.
 * Therefore, mode is set terminate to skip the loop. Full coverage is not achieved.
 */
TEST(arwain__FreeFuncs, test_lora_rx)
{
    arwain::system_mode = arwain::OperatingMode::Terminate;
    arwain::test_lora_rx();
}

TEST(arwain__FreeFuncs, test_inference)
{
    // Gets stuck in a loop somewhere and never returns, even with MockInferrer.
    EXPECT_EQ(arwain::ReturnCode::Success, arwain::test_inference());
}

TEST(arwain__FreeFuncs, test_mag)
{
    // TODO Incomplete coverage because chip ID returned by func is not 0x3D.
    EXPECT_EQ(arwain::ReturnCode::FailedMagnetometer, arwain::test_mag());
}

TEST(arwain__FreeFuncs, setup_log_directory)
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

TEST(arwain__FreeFuncs, execute_jobs)
{
    FAIL();
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

TEST(arwain__FreeFuncs, calibrate_magnetometers)
{
    FAIL();
    std::thread th = std::thread{
        []()
        {
            sleep_ms(3500);
            arwain::system_mode = arwain::OperatingMode::Terminate;
        }
    };
    EXPECT_EQ(arwain::calibrate_magnetometers(), arwain::ReturnCode::Success);
    th.join();
}

TEST(arwain__FreeFuncs, test_lora_tx)
{
    FAIL();
    arwain::test_lora_tx();
}

TEST(arwain__FreeFuncs, test_ori)
{
    EXPECT_EQ(arwain::ReturnCode::Success, arwain::test_ori(1));
}

double unwrap_phase_radians(double new_angle, double previous_angle);
TEST(FreeFuncs, unwrap_phase_radians)
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
TEST(FreeFuncs, unwrap_phase_degrees)
{
    double angle = 0;

    EXPECT_EQ(179, unwrap_phase_degrees(angle + 179, angle));
    EXPECT_EQ(181 - 360, unwrap_phase_degrees(angle + 181, angle));

    EXPECT_EQ(-179, unwrap_phase_degrees(angle - 179, angle));
    EXPECT_EQ(-181 + 360, unwrap_phase_degrees(angle - 181, angle));
}

std::string date_time_string();
TEST(FreeFuncs, date_time_string)
{
    EXPECT_NO_THROW(date_time_string());
}

TEST(arwain__FreeFuncs, compute_euler)
{
    Quaternion q{1, 0, 0, 0};
    EXPECT_NO_THROW(arwain::compute_euler(q));
}

TEST(arwain__FreeFuncs, apply_quat_rotor_to_vector3)
{
    Vector3 v{1, 0, 0};
    Quaternion q{1, 0, 0, 0};
    EXPECT_NO_THROW(arwain::apply_quat_rotor_to_vector3(v, q));
}

TEST(arwain__FreeFuncs, setup_log_folder_name_suffix__with_no_name)
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

TEST(arwain__FreeFuncs, setup_log_folder_name_suffix__with_name)
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

TEST(arwain__FreeFuncs, rerun_orientation_filter)
{
    // Create file to operate on
    std::ofstream acce_log{"./acce.txt"};
    std::ofstream gyro_log{"./gyro.txt"};

    acce_log << "headers aren't parsed so whatever\n";
    gyro_log << "headers aren't parsed so whatever\n";

    for (int i = 0; i < 10; i++)
    {
        acce_log << "0 0.0 0.0 9.81\n";
        gyro_log << "0 0.0 0.0 0.0\n";
    }

    acce_log.close();
    gyro_log.close();

    // Rerun the filter on the example file.
    auto ret = arwain::rerun_orientation_filter(".");
    EXPECT_EQ(ret, arwain::ReturnCode::Success);

    // SHould have created new files called slow_file, fast_file, fusion_file
    EXPECT_TRUE(std::filesystem::exists("./slow_file.txt"));
    EXPECT_TRUE(std::filesystem::exists("./fast_file.txt"));
    EXPECT_TRUE(std::filesystem::exists("./fusion_file.txt"));
}

TEST(arwain__FreeFuncs, rerun_floor_tracker)
{
    // Create file to operate on
    std::ofstream floor_log{"./position.txt"};
    floor_log << "headers aren't parsed so whatever\n";
    for (int i = 0; i < 10; i++)
    {
        floor_log << "0 0.0 0.0 0.0\n";
    }
    floor_log.close();

    // Run the floor tracker on the example file.
    auto ret = arwain::rerun_floor_tracker(".");
    EXPECT_EQ(ret, arwain::ReturnCode::Success);

    // Should have created a file called "pos_out.txt".
    EXPECT_TRUE(std::filesystem::exists("./pos_out.txt"));
}

TEST(arwain__FreeFuncs, calibrate_gyroscopes_offline)
{
    EXPECT_EQ(arwain::ReturnCode::Success, arwain::calibrate_gyroscopes_offline());
}

std::tuple<Vector3, Vector3> deduce_calib_params(const std::vector<Vector3>& readings);
TEST(arwain__FreeFuncs, deduce_calib_params)
{
    std::vector<Vector3> data
    {
        {0, 0, 1},
        {0, 0, -1},
        {0, 1, 0},
        {0, -1, 0},
        {1, 0, 0},
        {-1, 0, 0}
    };
    auto [bias, scale] = deduce_calib_params(data);
    EXPECT_EQ(bias, (Vector3{0, 0, 0}));
    EXPECT_EQ(scale, (Vector3{1, 1, 1}));
}

void sigint_handler(int signal);
TEST(FreeFuncs, sigint_handler)
{
    arwain::system_mode = arwain::OperatingMode::Idle;
    EXPECT_TRUE((arwain::system_mode == arwain::OperatingMode::Idle));
    sigint_handler(SIGINT);
    EXPECT_TRUE((arwain::system_mode == arwain::OperatingMode::Terminate));
}
