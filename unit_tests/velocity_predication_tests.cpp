#include <gtest/gtest.h>

#include "velocity_prediction.hpp"

TEST(Velocity_Prediction, Join)
{
    PositionVelocityInference inference;
    //inference.init();
    arwain::system_mode = arwain::OperatingMode::Terminate;
    EXPECT_NO_THROW(inference.join());
}

TEST(Velocity_Prediction, Init_Success)
{
    /*PositionVelocityInference inference;
    arwain::config.no_inference = false;
    EXPECT_TRUE(inference.init());
    arwain::system_mode = arwain::OperatingMode::Terminate;
    inference.join();*/
    FAIL();
}

TEST(Velocity_Prediction, Init_Failure)
{
    /*PositionVelocityInference inference;
    arwain::config.no_inference = true;
    EXPECT_FALSE(inference.init());
    arwain::system_mode = arwain::OperatingMode::Terminate;
    inference.join();*/
    FAIL();
}

TEST(Velocity_Prediction, Run_Inference)
{
    //PositionVelocityInference inference;
    //inference.run_idle();

    //arwain::Timers::CountdownTimer timer{100};
    //ASSERT_EQ(timer.finished(), false);
    //std::this_thread::sleep_for(std::chrono::milliseconds{100});
    //ASSERT_EQ(timer.finished(), true);
   FAIL();
}

TEST(Velocity_Prediction, Run_Idle)
{
    FAIL();
}

TEST(Velocity_Prediction, Core_Setup)
{
   FAIL();
}

TEST(Velocity_Prediction, Setup_Inference)
{
    //Can't be tested in its current state
    FAIL();
}

TEST(Velocity_Prediction, Run)
{
   FAIL();
}

TEST(Velocity_Prediction, Cleanup_Inference)
{
    FAIL();
}

#if USE_NCS2
TEST(Velocity_Prediction, Py_Inference)
{
    // TODO Improve testability of py_inference funciton.
    // This function just calls a Python script then returns. Nothing else is testable at this stage.
    // In fact it will probably fail to find the script, but we can't test that effectively.
    PositionVelocityInference inferrer;
    inferrer.py_inference();
    arwain::system_mode = arwain::OperatingMode::Terminate;
    inferrer.join();
    SUCCEED();
}
#endif