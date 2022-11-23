#include <gtest/gtest.h>

#include "arwain.hpp"
#include "input_parser.hpp"

TEST(Arwain, Arwain_Main)
{
    FAIL();
}

TEST(Arwain, Operator_Test)
{
    FAIL();
}

TEST(Arwain, Setup_No_Name)
{
    int j = 3;
    std::string program = "arwain_test";
    std::string command = "hello";
    std::string paramater = "1";
    char* input_array[3] = {program.data(),command.data(), paramater.data()};
    InputParser parser(j, input_array);

    arwain::setup(parser);
    std::string empty = "";

    EXPECT_EQ(empty, arwain::folder_date_string_suffix);
}

TEST(Arwain, Setup_Name)
{
    int j = 3;
    std::string program = "arwain_test";
    std::string command = "-name";
    std::string paramater = "example";
    char* input_array[3] = {program.data(),command.data(), paramater.data()};
    InputParser parser(j, input_array);

    arwain::setup(parser);
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
