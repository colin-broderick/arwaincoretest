#include <gtest/gtest.h>

#include "arwain/global_buffer.hpp"

TEST(GlobalBuffer, GlobalBuffer)
{
    EXPECT_NO_THROW((GlobalBuffer<int, 10>()));
}

TEST(GlobalBuffer, get_data)
{
    GlobalBuffer<int, 10> data;
    std::deque<int> expected_data;
    for (int i = 0; i < 10; i++)
    {
        expected_data.push_back(0);
    }
    EXPECT_EQ(data.get_data(), expected_data);
}

TEST(GlobalBuffer, push_back)
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

TEST(GlobalBuffer, back)
{
    GlobalBuffer<double, 100> data;
    data.push_back(445.345);
    data.push_back(4.35);
    data.push_back(45.45);
    data.push_back(5.3);
    EXPECT_EQ(data.back(), 5.3);
}

TEST(GlobalBuffer, clear)
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
