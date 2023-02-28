#include <gtest/gtest.h>

#include "uwb_reader.hpp"
#include "arwain.hpp"

#if USE_UUBLA
TEST(UWB_Reader, Init_Both_Conditions_False)
{
    //PositionVelocityInference inference;
    /*arwain::config.use_uwb_positioning = false;
    arwain::config.node_id = 1;
    EXPECT_FALSE(UublaWrapper::init());
    arwain::system_mode = arwain::OperatingMode::Terminate;
    UublaWrapper::join();*/

    FAIL(); //SEG FAULT TO BE SORTED FIRST!!!
}
#endif

#if USE_UUBLA
TEST(UWB_Reader, Init_Positioning_True)
{
    FAIL();
}
#endif

#if USE_UUBLA
TEST(UWB_Reader, Init_Node_ID_True)
{
    FAIL();
}
#endif

#if USE_UUBLA
TEST(UWB_Reader, Init_Both_Conditions_True)
{
    FAIL();
}
#endif

#if USE_UUBLA
TEST(UWB_Reader, Join)
{
    FAIL();
}
#endif

#if USE_UUBLA
TEST(UWB_Reader, Get_Distance)
{
    FAIL();
}
#endif
