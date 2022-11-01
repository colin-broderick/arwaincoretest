#include <gtest/gtest.h>

#include "vector3.hpp"

TEST(Vector3, Magnitude)
{
    Vector3 vector{1, 2, 3};

    EXPECT_EQ(vector.magnitude(), std::sqrt(1.0*1.0+2.0*2.0+3.0*3.0));
}
