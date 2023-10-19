#include <gtest/gtest.h>

std::streambuf* original_cout_buffer;
std::streambuf* original_cerr_buffer;

int main(int argc, char* argv[])
{
    original_cout_buffer = std::cout.rdbuf(nullptr);
    original_cerr_buffer = std::cerr.rdbuf(nullptr);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
