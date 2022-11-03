#include <gtest/gtest.h>

#include "input_parser.hpp"

/** \brief Creates a parser object with a given command, then checks command has been stored correctly.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(InputParser, Constructor)
{
    int j = 2;
    std::string program = "arwain_test";
    std::string command = "hello";
    char* input_array[2] = {program.data(),command.data()};
    InputParser parser(j, input_array);
    
    EXPECT_TRUE(parser.contains("hello"));
}

/** \brief Creates a parser object with a given command and a value for that command, then checks the command value has been stored correctly
 *  and can be retreived.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(InputParser, GetCmdOption)
{
    int j = 3;
    std::string program = "arwain_test";
    std::string command = "hello";
    std::string paramater = "1";
    char* input_array[3] = {program.data(),command.data(), paramater.data()};
    InputParser parser(j, input_array);

    EXPECT_EQ(parser.getCmdOption("hello"), "1");
}

/** \brief Creates a parser object with a given command, then checks that there hasn't been a value assosiated with that command. 
 * \return 0 for test pass, 1 for test fail.
 */
TEST(InputParser, GetCmdOptionError)
{
    int j = 2;
    std::string program = "arwain_test";
    std::string command = "hello";
    char* input_array[2] = {program.data(),command.data()};
    InputParser parser(j, input_array);

    EXPECT_EQ(parser.getCmdOption("hello"), "");
}

/** \brief Creates a parser object with a given command, then checks for a different command to check that it is not present.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(InputParser, ContainerError)
{
    int j = 2;
    std::string program = "arwain_test";
    std::string command = "hello";
    char* input_array[2] = {program.data(),command.data()};
    InputParser parser(j, input_array);

    EXPECT_FALSE(parser.contains("bye"));
}