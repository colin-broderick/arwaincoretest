#include <gtest/gtest.h>

#if USE_UUBLA

#include "uwb_reader.hpp"
#include "arwain.hpp"

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

TEST(UWB_Reader, Init_Positioning_True)
{
    FAIL();
}

TEST(UWB_Reader, Init_Node_ID_True)
{
    FAIL();
}

TEST(UWB_Reader, Init_Both_Conditions_True)
{
    FAIL();
}

TEST(UWB_Reader, Join)
{
    FAIL();
}

TEST(UWB_Reader, Get_Distance)
{
    FAIL();
}
<<<<<<< HEAD
=======

#endif
>>>>>>> f96480aedb135fca5fd5bca44300b09f28a5d5a9
