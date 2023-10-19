#include <gtest/gtest.h>

// These pointers retained as some legacy tests still use them.
// New tests should print to stringstream or similar instead.
std::streambuf* original_cout_buffer;
std::streambuf* original_cerr_buffer;

int main(int argc, char* argv[])
{
    original_cout_buffer = std::cout.rdbuf(nullptr);
    original_cerr_buffer = std::cerr.rdbuf(nullptr);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
