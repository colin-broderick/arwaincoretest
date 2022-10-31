#include <map>
#include <random>
#include <iostream>
#include <functional>
#include <fstream>
#include <filesystem>

#include <gtest/gtest.h>

#include "input_parser.hpp"
#include "logger.hpp"

namespace Random
{
    /** \brief Generates a random double in the interval [-10, 10).
     * \return Double in [-10, 10).
     */
    static double Double()
    {
        static std::random_device rd;
        static std::mt19937 mt(rd());
        static std::uniform_real_distribution<double> dist(-10.0, 10.0);
        return dist(mt);
    }

    /** \brief Generates a random double in the interval [lower_bound, upper_bound). 
     * \param lower_bound Smallest value that can be returned.
     * \param upper_bound Lowest upper bound on returnable values.
     * \return Double in [lower_bound, upper_bound).
     */
    static double DoubleBetween(const double lower_bound, const double upper_bound)
    {
        static std::random_device rd;
        static std::mt19937 mt(rd());
        std::uniform_real_distribution<double> dist(lower_bound, upper_bound);
        return dist(mt);
    }
}

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

/** \brief Creates a file object, checks file has been created correctly.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(Logger, CreateFile)
{
    arwain::Logger file("test_file");

    EXPECT_EQ(file.get_filename(), "test_file");
    EXPECT_TRUE(file.is_open());

    file.close();
    std::filesystem::remove("test_file");
}

/** \brief Creates a file object, checks file object is empty.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(Logger, CreateFileEmpty)
{
    arwain::Logger file;

    EXPECT_EQ(file.get_filename(), "");
    EXPECT_FALSE(file.is_open());

    std::filesystem::remove("test_file");
}

/** \brief checks is_open function is correct.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(Logger, FileIsOpen)
{
    arwain::Logger file("test_file");

    EXPECT_TRUE(file.is_open());
    file.close();
    std::filesystem::remove("test_file");
}

/** \brief checks is_open function is correct.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(Logger, FileClosed)
{
    arwain::Logger file("test_file");
    file.close();

    EXPECT_FALSE(file.is_open());
    EXPECT_TRUE(std::filesystem::exists("test_file"));

    std::filesystem::remove("test_file");
}

/** \brief checks get_filename() function is correct.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(Logger, GetFileName)
{
    arwain::Logger file("test_file");
    file.close();

    EXPECT_EQ(file.get_filename(), "test_file");
    std::filesystem::remove("test_file");
}

/** \brief checks open() function is correct.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(Logger, OpenFile)
{
    arwain::Logger file("test_file");
    file.close();

    EXPECT_TRUE(file.open("test_file"));

    file.close();
    std::filesystem::remove("test_file");
}

/** \brief checks open() function is correct when file is already open.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(Logger, FileAlreadyOpen)
{
    arwain::Logger file("test_file");

    EXPECT_FALSE(file.open("test_file"));
    file.close();
    std::filesystem::remove("test_file");
}

/** \brief checks << operator inserts data into a file correctly.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(Logger, InsertString)
{
    arwain::Logger file("test_file");
    
    file << "test string!";
    file.close();

    std::ifstream inputfile{"test_file"};
    std::string data;
    std::getline(inputfile, data);
    std::filesystem::remove("test_file");

    EXPECT_EQ(data, "test string!");
}

/** \brief checks << operator inserts data into a file correctly.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(Logger, InsertInt)
{
    arwain::Logger file("test_file");
    
    file << 1;
    file.close();

    std::ifstream inputfile{"test_file"};
    std::string data;
    std::getline(inputfile, data);
    std::filesystem::remove("test_file");

    EXPECT_EQ(data, "1");
}

/** \brief checks << operator inserts data into a file correctly.
 * \return 0 for test pass, 1 for test fail.
 */
TEST(Logger, InsertFloat)
{
    arwain::Logger file("test_file");
    
    file << 1.1;
    file.close();

    std::ifstream inputfile{"test_file"};
    std::string data;
    std::getline(inputfile, data);
    std::filesystem::remove("test_file");

    EXPECT_EQ(data, "1.1");
}

int main(int argc, char* argv[])
{
    std::cout.rdbuf(nullptr);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
