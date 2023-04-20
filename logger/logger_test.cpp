#include <gtest/gtest.h>
#include <fstream>

#include "logger.hpp"

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
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
