#include <gtest/gtest.h>

int main(int argc, char* argv[])
{
    std::cout.rdbuf(nullptr);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
