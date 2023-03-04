#include <gtest/gtest.h>
#include <string>
#include <string_view>
#include <iostream>

#include "command_line.hpp"
#include "velocity_prediction.hpp"

TEST(ArwainCLI, run)
{
    FAIL();
}

// TODO Most of the tests here are not properly exited, so you'll get random
// segfaults and core dumps from threads not being properly joined.

TEST(ArwainCLI, s2i__exit)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("exit\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    command_line.join();
    EXPECT_EQ(command_line.s2i("exit"), 0);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__stop)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("stop\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;


    command_line.join();
    EXPECT_EQ(command_line.s2i("stop"), 0);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__shutdown)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("shutdown\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    command_line.join();
    EXPECT_EQ(command_line.s2i("shutdown"), 0);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__quit)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("quit\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    command_line.join();
    EXPECT_EQ(command_line.s2i("quit"), 0);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__infer)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("infer\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("infer"), 1);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__inference)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("inference\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("inference"), 1);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__autocal)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("autocal\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("autocal"), 2);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__idle)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("idle\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("idle"), 2);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__mode)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("mode\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("mode"), 3);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__calibg)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("calibg\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("calibg"), 4);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__calibm)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("calibm\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("calibm"), 6);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__caliba)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("caliba\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("caliba"), 5);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__record)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("record\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("record"), 9);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__name)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("name\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("name"), 7);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__help)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("help\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("help"), 10);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__zeropos)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("zeropos\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("zeropos"), 11);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__name_substr)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("name_test\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("name_test"), 7);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, s2i__default)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("test\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;

    arwain::system_mode = arwain::OperatingMode::Terminate;

    command_line.join();
    EXPECT_EQ(command_line.s2i("test"), 8);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, switch_to_exit_mode)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("test\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;
    command_line.switch_to_exit_mode();
    command_line.join();
    EXPECT_TRUE(arwain::system_mode == arwain::OperatingMode::Terminate);
    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, report_current_mode)
{
    // so simple that it probably can't be meaningfully tests
    FAIL();
}

TEST(ArwainCLI, switch_to_idle_autocal_mode__from_inference)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("test\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;
    arwain::system_mode = arwain::OperatingMode::Terminate;
    command_line.join();

    arwain::system_mode = arwain::OperatingMode::Inference;

    command_line.switch_to_idle_autocal_mode();
    EXPECT_TRUE(arwain::system_mode == arwain::OperatingMode::Idle);

    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, switch_to_idle_autocal_mode__from_something_else)
{
    /*std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("test\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;
    arwain::system_mode = arwain::OperatingMode::Terminate;
    command_line.join();

    arwain::system_mode = arwain::OperatingMode::Inference;

    command_line.switch_to_idle_autocal_mode();
    EXPECT_TRUE(arwain::system_mode == arwain::OperatingMode::Idle);

    std::cin.rdbuf(orig);*/

    // waiting for refactor
    FAIL();
}

TEST(ArwainCLI, switch_to_inference_mode)
{
    FAIL();
}

TEST(ArwainCLI, core_setup)
{
    FAIL();
}

TEST(ArwainCLI, force_switch_to_idle_autocal_mode)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("test\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI command_line;
    arwain::system_mode = arwain::OperatingMode::Terminate;
    command_line.join();

    command_line.force_switch_to_idle_autocal_mode();
    EXPECT_TRUE(arwain::system_mode == arwain::OperatingMode::Idle);

    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, switch_to_gyro_calib_mode)
{
    FAIL();
}

TEST(ArwainCLI, fail_to_switch_to)
{
    FAIL();
}

TEST(ArwainCLI, switch_to_mag_calib_mode)
{
    FAIL();
}

TEST(ArwainCLI, switch_to_accel_calib_mode)
{
    FAIL();
}

TEST(ArwainCLI, switch_to_data_collection_mode)
{
    FAIL();
}

TEST(ArwainCLI, set_folder_name)
{
    arwain::config.no_cli = true;
    arwain::system_mode = arwain::OperatingMode::Inference;
    ArwainCLI cli;

    std::string pre_value = arwain::folder_date_string_suffix;

    // Should have no effect in inference mode.
    cli.set_folder_name("name test_folder_name");
    std::string post_value = arwain::folder_date_string_suffix;
    EXPECT_EQ(pre_value, post_value);

    // Should be ineffective with invalid input.
    cli.set_folder_name("name name");
    cli.set_folder_name("name");
    post_value = arwain::folder_date_string_suffix;
    EXPECT_EQ(pre_value, post_value);

    // Should be effective in idle mode.
    arwain::system_mode = arwain::OperatingMode::Idle;
    cli.set_folder_name("name test_folder_name");
    post_value = "test_folder_name";
    EXPECT_EQ(post_value, arwain::folder_date_string_suffix);
}

/** \brief Produces no easily testable output. */
TEST(ArwainCLI, parse_cli_input)
{
    FAIL();
}

TEST(ArwainCLI, set_velocity_inference_pointer)
{
    arwain::config.no_inference = true;
    PositionVelocityInference inferrer;
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("exit\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI cli;

    EXPECT_EQ(cli.velocity_inference_handle, nullptr);

    EXPECT_TRUE(cli.set_velocity_inference_pointer(inferrer));

    EXPECT_NE(cli.velocity_inference_handle, nullptr);

    cli.join();
    inferrer.join();

    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, ArwainCLI)
{
    FAIL();
}

/** \brief We construct a CLI, then shut it down, then call init manually and check for expected state. */
TEST(ArwainCLI, init)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("exit\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI cli;
    EXPECT_TRUE(cli.job_thread.joinable());
    cli.join();

    EXPECT_TRUE(cli.init());
    std::cin.rdbuf(input.rdbuf());
    EXPECT_TRUE(cli.job_thread.joinable());
    cli.join();

    std::cin.rdbuf(orig);
}

TEST(ArwainCLI, join)
{
    std::streambuf *orig = std::cin.rdbuf();
    std::istringstream input("exit\n");
    std::cin.rdbuf(input.rdbuf());
    ArwainCLI cli;
    EXPECT_TRUE(cli.job_thread.joinable());
    cli.join();

    std::cin.rdbuf(orig);
}
