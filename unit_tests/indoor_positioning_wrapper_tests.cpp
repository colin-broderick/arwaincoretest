#include <gtest/gtest.h>

#include "indoor_positioning_wrapper.hpp"
//#include "floor_tracker.hpp"
//#include "corner_detector.hpp"
//#include "logger.hpp"
//#include "vector3.hpp"
#include "arwain.hpp"

//public functions
TEST(Indoor_Positioning_Public, Init_Success)
{
    /*IndoorPositioningSystem pos_system;
    arwain::config.use_ips = true;
    EXPECT_TRUE(pos_system.init());
    arwain::system_mode = arwain::OperatingMode::Terminate;
    pos_system.join();*/

    FAIL();
}

TEST(Indoor_Positioning_Public, Init_Failure)
{
    /*IndoorPositioningSystem pos_system;
    arwain::config.use_ips = false;
    EXPECT_FALSE(pos_system.init());
    arwain::system_mode = arwain::OperatingMode::Terminate;
    pos_system.join();*/

    FAIL();
}

TEST(Indoor_Positioning_Public, Join)
{
    IndoorPositioningSystem pos_system;
    //pos_system.init();
    arwain::system_mode = arwain::OperatingMode::Terminate;
    EXPECT_NO_THROW(pos_system.join());
}

//private functions
TEST(Indoor_Positioning_Private, Core_Setup)
{
    FAIL(); // function not implemented yet!
}

TEST(Indoor_Positioning_Private, Setup_Inference)
{
    FAIL(); // function not implemented yet!
}

TEST(Indoor_Positioning_Private, Cleanup_Inference)
{
    FAIL(); // function not implemented yet!
}

TEST(Indoor_Positioning_Private, Run_Idle)
{
    FAIL(); // function not implemented yet!
}

TEST(Indoor_Positioning_Private, Run_Inference)
{
    FAIL(); // function not implemented yet!
}

TEST(Indoor_Positioning_Private, Run)
{
    FAIL(); // function not implemented yet!
}

//public wrapper functions
TEST(Indoor_Positioning_Wrapper, Update)
{
    FAIL(); // function not implemented yet!
}

TEST(Indoor_Positioning_Wrapper, getPosition)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    Vector3 pos{1.0, 1.0, 1.0};

    EXPECT_EQ(pos, wrapper.getPosition());
}

TEST(Indoor_Positioning_Wrapper, getX)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    double x = 1.0;
    EXPECT_EQ(x, wrapper.getX());
}

TEST(Indoor_Positioning_Wrapper, getY)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    double y = 1.0;
    EXPECT_EQ(y, wrapper.getY());
}

TEST(Indoor_Positioning_Wrapper, getZ)
{
    IndoorPositioningSystem::IndoorPositioningWrapper wrapper;
    double z = 1.0;
    EXPECT_EQ(z, wrapper.getZ());
}
