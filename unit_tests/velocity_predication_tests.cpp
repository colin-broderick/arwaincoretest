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

/** \brief After the cleanup_inference() call, position and velocity
 * should both be zero-vector regardless of what they were before.
 * Log files should both be closed.
 */
TEST(Velocity_Prediction, Cleanup_Inference)
{
    PositionVelocityInference inferrer;
    inferrer.position = {1, 1, 1};
    inferrer.velocity = {1, 1, 1};
    EXPECT_EQ(inferrer.position, (Vector3{1, 1, 1}));
    EXPECT_EQ(inferrer.velocity, (Vector3{1, 1, 1}));
    inferrer.position_file.open("/dev/null");
    inferrer.velocity_file.open("/dev/null");
    EXPECT_TRUE(inferrer.position_file.is_open());
    EXPECT_TRUE(inferrer.velocity_file.is_open());

    inferrer.cleanup_inference();

    EXPECT_EQ(inferrer.position, (Vector3{0, 0, 0}));
    EXPECT_EQ(inferrer.velocity, (Vector3{0, 0, 0}));
    EXPECT_FALSE(inferrer.position_file.is_open());
    EXPECT_FALSE(inferrer.velocity_file.is_open());

    arwain::system_mode = arwain::OperatingMode::Terminate;
    inferrer.join();
}

#if USE_NCS2
/** \brief This function just calls a Python script then returns.
 * Nothing else is testable at this stage. In fact it will probably
 * fail to find the script, but we can't test that effectively.
 * 
 * TODO Improve testability of py_inference funciton.
 */
TEST(Velocity_Prediction, Py_Inference)
{
    PositionVelocityInference inferrer;
    inferrer.py_inference();
    arwain::system_mode = arwain::OperatingMode::Terminate;
    inferrer.join();
    SUCCEED();
}
#endif