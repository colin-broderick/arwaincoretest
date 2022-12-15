#include <gtest/gtest.h>
#include <chrono>

#include "arwain.hpp"
#include "input_parser.hpp"

TEST(ArwainUtils, clamp_value)
{
    int value = -5;
    EXPECT_EQ(0, clamp_value(value, 0, 17));
    EXPECT_EQ(-5, clamp_value(value, -17, 11));
    EXPECT_EQ(-10, clamp_value(value, -17, -10));
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

TEST(Arwain, Arwain_Main)
{
    FAIL();
}

TEST(Arwain, Operator_Test)
{
    FAIL();
}

TEST(Arwain, setup_log_folder_name_suffix_with_no_name)
{
    int j = 3;
    std::string program = "arwain_test";
    std::string command = "hello";
    std::string paramater = "1";
    char* input_array[3] = {program.data(),command.data(), paramater.data()};
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
    std::string paramater = "example";
    char* input_array[3] = {program.data(),command.data(), paramater.data()};
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
