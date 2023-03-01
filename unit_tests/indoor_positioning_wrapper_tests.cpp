#include <gtest/gtest.h>

#include "indoor_positioning_wrapper.hpp"
#include "arwain.hpp"

/** \brief After running init() successfully, the job thread should be running and joinable. */
TEST(Indoor_Positioning_Public, Init_Success)
{
    IndoorPositioningSystem ips;
    arwain::config.use_ips = true;
    EXPECT_TRUE(ips.init());
    arwain::system_mode = arwain::OperatingMode::Terminate;
    ips.join();
}

TEST(Indoor_Positioning_Public, Init_Failure)
{
    arwain::config.use_ips = false;
    IndoorPositioningSystem ips;
    EXPECT_FALSE(ips.init());
}

TEST(Indoor_Positioning_Public, Join)
{
    IndoorPositioningSystem pos_system;
    arwain::system_mode = arwain::OperatingMode::Terminate;
    EXPECT_NO_THROW(pos_system.join());
}

TEST(NOTREADY_Indoor_Positioning_Private, Core_Setup)
{
    FAIL(); // function not implemented yet!
}

TEST(NOTREADY_Indoor_Positioning_Private, Setup_Inference)
{
    FAIL(); // function not implemented yet!
}

TEST(NOTREADY_Indoor_Positioning_Private, Cleanup_Inference)
{
    FAIL(); // function not implemented yet!
}

TEST(NOTREADY_Indoor_Positioning_Private, Run_Idle)
{
    FAIL(); // function not implemented yet!
}

TEST(NOTREADY_Indoor_Positioning_Private, Run_Inference)
{
    FAIL(); // function not implemented yet!
}

TEST(NOTREADY_Indoor_Positioning_Private, Run)
{
    FAIL(); // function not implemented yet!
}

//public wrapper functions
TEST(NOTREADY_Indoor_Positioning_Wrapper, Update)
{
    FAIL(); // function not implemented yet!
}

TEST(Indoor_Positioning_Wrapper, getPosition)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    wrapper.m_x = 1.0f;
    wrapper.m_y = 1.0f;
    wrapper.m_z = 1.0f;

    Vector3 pos{1.0, 1.0, 1.0};

    EXPECT_EQ(pos, wrapper.getPosition());
}

TEST(Indoor_Positioning_Wrapper, getX)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    wrapper.m_x = 1.0f;
    double x = 1.0;
    EXPECT_EQ(x, wrapper.getX());
}

TEST(Indoor_Positioning_Wrapper, getY)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    wrapper.m_y = 1.0f;
    double y = 1.0;
    EXPECT_EQ(y, wrapper.getY());
}

TEST(Indoor_Positioning_Wrapper, getZ)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    wrapper.m_z = 1.0f;
    double z = 1.0;
    EXPECT_EQ(z, wrapper.getZ());
}
