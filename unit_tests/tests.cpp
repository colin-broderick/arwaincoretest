#include <gtest/gtest.h>

// These pointers retained as some legacy tests still use them.
// New tests should print to stringstream or similar instead.

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
