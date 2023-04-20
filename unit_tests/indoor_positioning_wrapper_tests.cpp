#include <gtest/gtest.h>

#include "indoor_positioning_wrapper.hpp"
#include "arwain.hpp"

/** \brief After running init() successfully, the job thread should be running and joinable. */
TEST(IndoorPositioningSystem, init__success)
{
    FAIL();
    IndoorPositioningSystem ips;
    arwain::config.use_ips = true;
    EXPECT_TRUE(ips.init());
    EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    ips.join();
}

TEST(IndoorPositioningSystem, init__failure)
{
    FAIL();
    arwain::config.use_ips = false;
    IndoorPositioningSystem ips;
    EXPECT_FALSE(ips.init());
}

TEST(IndoorPositioningSystem, join)
{
    arwain::config.use_ips = true;
    IndoorPositioningSystem pos_system;
    EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    EXPECT_NO_THROW(pos_system.join());
}

TEST(IndoorPositioningSystem, core_setup)
{
    FAIL(); // function not implemented yet!
}

TEST(IndoorPositioningSystem, setup_inference)
{
    FAIL(); // function not implemented yet!
}

TEST(IndoorPositioningSystem, cleanup_inference)
{
    FAIL(); // function not implemented yet!
}

TEST(IndoorPositioningSystem, run_idle)
{
    FAIL(); // function not implemented yet!
}

TEST(IndoorPositioningSystem, run_inference)
{
    FAIL(); // function not implemented yet!
}

TEST(IndoorPositioningSystem, run)
{
    FAIL(); // function not implemented yet!
}

//public wrapper functions
TEST(IndoorPositioningWrapper, udpate)
{
    FAIL(); // function not implemented yet!
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
