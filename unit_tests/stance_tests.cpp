#include <gtest/gtest.h>

#include "stance.hpp"
#include "arwain.hpp"

TEST(StanceDetection, StanceDetection)
{
    EXPECT_NO_THROW(
        StanceDetection stance;
        arwain::system_mode = arwain::OperatingMode::Terminate;
        stance.join();
    );
}

TEST(StanceDetection, setup_inference)
{
    arwain::config.no_stance = true;
    arwain::folder_date_string = ".";
    StanceDetection stance;
    EXPECT_NO_THROW(stance.setup_inference());
    EXPECT_TRUE(stance.stance_file.is_open());
    stance.cleanup_inference();
    EXPECT_FALSE(stance.stance_file.is_open());
    stance.join();
}

TEST(StanceDetection, run_inference)
{
    arwain::config.no_stance = true;
    arwain::folder_date_string = ".";
    StanceDetection stance;
    arwain::Buffers::IMU_BUFFER.push_back({{0, 0, 0}, {0, 0, 0}});
    arwain::Buffers::VELOCITY_BUFFER.push_back({0, 0, 0});
    arwain::Buffers::QUAT_ORIENTATION_BUFFER.push_back({1, 0, 0, 0});
    arwain::system_mode = arwain::OperatingMode::Inference;
    stance.core_setup(); // Makes sure the internal stance detector is created.
    std::thread th = std::thread{
        []()
        {
            sleep_ms(500);
            arwain::system_mode = arwain::OperatingMode::Terminate;
        }
    };
    EXPECT_NO_THROW(stance.run_inference());
    th.join();
    stance.join();
}

TEST(StanceDetector, register_freefall_event)
{
    StanceDetector stance;
    EXPECT_EQ(stance.m_falling, StanceDetector::NotFalling);
    stance.register_freefall_event();
    EXPECT_EQ(stance.m_falling, StanceDetector::Falling);
}

TEST(StanceDetector, entangled_or_operator)
{
    auto stance1 = StanceDetector::EntangleState::Entangled;
    auto stance2 = StanceDetector::EntangleState::NotEntangled;
    EXPECT_EQ(stance1 | stance1, stance1);
    EXPECT_EQ(stance1 | stance2, stance1);
    EXPECT_EQ(stance2 | stance1, stance1);
    EXPECT_EQ(stance2 | stance2, stance2);
}

TEST(StanceDetector, fall_or_operator)
{
    auto stance1 = StanceDetector::FallState::Falling;
    auto stance2 = StanceDetector::FallState::NotFalling;
    EXPECT_EQ(stance1 | stance1, stance1);
    EXPECT_EQ(stance1 | stance2, stance1);
    EXPECT_EQ(stance2 | stance1, stance1);
    EXPECT_EQ(stance2 | stance2, stance2);
}

TEST(StanceDetector, get_means)
{
    {
        // deque of Vector6
        std::deque<std::array<double, 6>> data;
        data.push_back({1, 1, 1, 1, 1, 1});
        data.push_back({2, 2, 2, 2, 2, 2});
        StanceDetector stance;
        auto result = stance.get_means(data);
        EXPECT_EQ(result, (std::array<double, 6>{1.5, 1.5, 1.5, 1.5, 1.5, 1.5}));
    }
    {
        // deque of vector3
        std::deque<Vector3> data;
        data.push_back({1, 1, 1});
        data.push_back({2, 2, 2});
        StanceDetector stance;
        auto result = stance.get_means(data);
        EXPECT_EQ(result, (Vector3{1.5, 1.5, 1.5}));
    }
    {
        // Vector of Vector3
        std::vector<Vector3> data;
        data.push_back({1, 1, 1});
        data.push_back({2, 2, 2});
        StanceDetector stance;
        auto result = stance.get_means(data);
        EXPECT_EQ(result, (Vector3{1.5, 1.5, 1.5}));
    }
}

TEST(StanceDetector, buffer_mean_magnitude)
{
    std::deque<Vector3> data;
    data.push_back({1, 1, 1});
    data.push_back({2, 2, 2});
    double mean_magnitude = (std::sqrt(3) + std::sqrt(12)) / 2.0;
    StanceDetector stance;
    auto result = stance.buffer_mean_magnitude(data);
    EXPECT_EQ(result, mean_magnitude);
}

TEST(StanceDetector, vector_mean)
{
    std::vector<double> data{1, 2, 3, 4, 5};
    double mean = 3;
    StanceDetector stance;
    double result = stance.vector_mean(data);
    EXPECT_EQ(result, mean);
}

TEST(StanceDetector, biggest_axis)
{
    StanceDetector stance;
    Vector3 data1{1, 2, 3};
    Vector3 data2{1, 4, 3};
    Vector3 data3{5, 2, 3};
    EXPECT_EQ(stance.biggest_axis(data1), StanceDetector::Axis::ZAxis);
    EXPECT_EQ(stance.biggest_axis(data2), StanceDetector::Axis::YAxis);
    EXPECT_EQ(stance.biggest_axis(data3), StanceDetector::Axis::XAxis);
}

TEST(StanceDetector, get_stance)
{
    StanceDetector stance;
    stance.m_stance = StanceDetector::Stance::Climbing;
    EXPECT_EQ(stance.get_stance(), StanceDetector::Stance::Climbing);
    stance.m_stance = StanceDetector::Stance::Crawling;
    EXPECT_EQ(stance.get_stance(), StanceDetector::Stance::Crawling);
    stance.m_stance = StanceDetector::Stance::Inactive;
    EXPECT_EQ(stance.get_stance(), StanceDetector::Stance::Inactive);
    stance.m_stance = StanceDetector::Stance::Prone;
    EXPECT_EQ(stance.get_stance(), StanceDetector::Stance::Prone);
    stance.m_stance = StanceDetector::Stance::Running;
    EXPECT_EQ(stance.get_stance(), StanceDetector::Stance::Running);
    stance.m_stance = StanceDetector::Stance::Searching;
    EXPECT_EQ(stance.get_stance(), StanceDetector::Stance::Searching);
    stance.m_stance = StanceDetector::Stance::Walking;
    EXPECT_EQ(stance.get_stance(), StanceDetector::Stance::Walking);
}

TEST(StanceDetector, get_attitude)
{
    StanceDetector stance;
    stance.m_attitude = StanceDetector::Attitude::Horizontal;
    EXPECT_EQ(stance.get_attitude(), StanceDetector::Attitude::Horizontal);
    stance.m_attitude = StanceDetector::Attitude::Vertical;
    EXPECT_EQ(stance.get_attitude(), StanceDetector::Attitude::Vertical);
}

TEST(StanceDetector, get_entangled_state)
{
    StanceDetector stance;
    stance.m_entangled = StanceDetector::EntangleState::Entangled;
    EXPECT_EQ(stance.get_entangled_status(), StanceDetector::EntangleState::Entangled);
    stance.m_entangled = StanceDetector::EntangleState::NotEntangled;
    EXPECT_EQ(stance.get_entangled_status(), StanceDetector::EntangleState::NotEntangled);
}

TEST(StanceDetector, get_falling_status)
{
    StanceDetector stance;
    stance.m_falling = StanceDetector::FallState::Falling;
    EXPECT_EQ(stance.get_falling_status(), StanceDetector::FallState::Falling);
    stance.m_falling = StanceDetector::FallState::NotFalling;
    EXPECT_EQ(stance.get_falling_status(), StanceDetector::FallState::NotFalling);
}
