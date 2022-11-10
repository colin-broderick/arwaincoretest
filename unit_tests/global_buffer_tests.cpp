#include <gtest/gtest.h>

#include "global_buffer.hpp"

TEST(GlobalBuffer, Constructor)
{
    EXPECT_NO_THROW((GlobalBuffer<int, 10>()));
}

TEST(GlobalBuffer, GetData)
{
    GlobalBuffer<int, 10> data;
    std::deque<int> expected_data;
    for (int i = 0; i < 10; i++)
    {
        expected_data.push_back(0);
    }
    EXPECT_EQ(data.get_data(), expected_data);
}

TEST(GlobalBuffer, PushBackData)
{
    GlobalBuffer<int, 10> data;
    std::deque<int> expected_data;
    for (int i = 0; i < 9; i++)
    {
        expected_data.push_back(0);
    }
    expected_data.push_back(1);

    data.push_back(1);

    EXPECT_EQ(data.get_data(), expected_data);
    EXPECT_EQ(data.get_data().size(), 10);
}

TEST(GlobalBuffer, Back)
{
    GlobalBuffer<double, 100> data;
    data.push_back(445.345);
    data.push_back(4.35);
    data.push_back(45.45);
    data.push_back(5.3);
    EXPECT_EQ(data.back(), 5.3);
}

TEST(GlobalBuffer, Clear)
{
    GlobalBuffer<double, 100> data;
    data.push_back(445.345);
    data.push_back(4.35);
    data.push_back(45.45);
    data.push_back(5.3);
    data.clear();

    std::deque<double> expected_data(100);
    EXPECT_EQ(expected_data, data.get_data());
}
