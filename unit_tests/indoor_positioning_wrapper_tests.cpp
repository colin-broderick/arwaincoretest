#include <gtest/gtest.h>

#include "arwain/indoor_positioning_wrapper.hpp"
#include "arwain/arwain.hpp"
#include "arwain/events.hpp"

TEST(IndoorPositioningSystem, join)
{
    arwain::config.use_ips = true;
    IndoorPositioningSystem pos_system;
    arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    EXPECT_NO_THROW(pos_system.join());
}

TEST(IndoorPositioningSystem, core_setup)
{
    GTEST_SKIP(); // function not implemented yet!
}

TEST(IndoorPositioningSystem, setup_inference)
{
    GTEST_SKIP(); // function not implemented yet!
}

TEST(IndoorPositioningSystem, cleanup_inference)
{
    GTEST_SKIP(); // function not implemented yet!
}

TEST(IndoorPositioningSystem, run_idle)
{
    GTEST_SKIP(); // function not implemented yet!
}

TEST(IndoorPositioningSystem, run_inference)
{
    GTEST_SKIP(); // function not implemented yet!
}

TEST(IndoorPositioningSystem, run)
{
    GTEST_SKIP(); // function not implemented yet!
}

//public wrapper functions
TEST(IndoorPositioningWrapper, udpate)
{
    GTEST_SKIP(); // function not implemented yet!
}

TEST(IndoorPositioningWrapper, get_position)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    wrapper.m_x = 1.0f;
    wrapper.m_y = 1.0f;
    wrapper.m_z = 1.0f;

    Vector3 pos{1.0, 1.0, 1.0};

    EXPECT_EQ(pos, wrapper.get_position());
}

TEST(IndoorPositioningWrapper, get_x)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    wrapper.m_x = 1.0f;
    double x = 1.0;
    EXPECT_EQ(x, wrapper.get_x());
}

TEST(IndoorPositioningWrapper, get_y)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    wrapper.m_y = 1.0f;
    double y = 1.0;
    EXPECT_EQ(y, wrapper.get_y());
}

TEST(IndoorPositioningWrapper, get_z)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    wrapper.m_z = 1.0f;
    double z = 1.0;
    EXPECT_EQ(z, wrapper.get_z());
}
