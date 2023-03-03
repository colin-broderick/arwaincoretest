#include <gtest/gtest.h>

#include "velocity_prediction.hpp"

TEST(MockInferrer, init_infer)
{
    std::deque<ImuData> data;
    data.push_back({{0, 0, 0}, {0, 0, 0}});
    MockInferrer inferrer;
    EXPECT_NO_THROW(inferrer.init());
    EXPECT_EQ(inferrer.infer(data), (Vector3{0, 0, 0}));
}

TEST(Velocity_Prediction, Join)
{
    PositionVelocityInference inference;
    //inference.init();
    arwain::system_mode = arwain::OperatingMode::Terminate;
    EXPECT_NO_THROW(inference.join());
}

/** \brief After init(), the job_thread(s) should be running and should be
 * joinable but incomplete. */
TEST(Velocity_Prediction, Init_Success)
{
    arwain::config.no_inference = true;
    PositionVelocityInference inferrer;

    // At this point, the inferrer exists, but init() has not been fully executed.
    // and the job thread(s) is not created.
    arwain::config.no_inference = false;
    EXPECT_TRUE(inferrer.init());

    EXPECT_TRUE(inferrer.job_thread.joinable());
    #if USE_NCS2
    EXPECT_TRUE(inferrer.job_thread.joinable());
    #endif

    arwain::system_mode = arwain::OperatingMode::Terminate;
    inferrer.join();
}

/** \brief If the config.no_inference option is on, then init() should return
 * immediately with a false having taken no action. The job_thread should not
 * be joinable because it should never have started. If the ncs2 option is on,
 * it has its own thread which should also not be joinable.
 */
TEST(Velocity_Prediction, Init_Failure)
{
    arwain::config.no_inference = true;
    PositionVelocityInference inference;
    EXPECT_FALSE(inference.init());
    EXPECT_FALSE(inference.job_thread.joinable());
    #if USE_NCS2
    EXPECT_FALSE(inference.ncs2_thread.joinable());
    #endif
    arwain::system_mode = arwain::OperatingMode::Terminate;
    inference.join();
}

TEST(HARDWARE_NOTREADY_Velocity_Prediction, Run_Inference)
{
    //PositionVelocityInference inference;
    //inference.run_idle();

    //arwain::Timers::CountdownTimer timer{100};
    //ASSERT_EQ(timer.finished(), false);
    //std::this_thread::sleep_for(std::chrono::milliseconds{100});
    //ASSERT_EQ(timer.finished(), true);
   FAIL();
}

/** \brief The run_idle function is a no-op other than a sleep of some unknown duration.
 * Since the duration is unknown, all we can really do is check that it doesn't generate
 * any errors.
 */
TEST(Velocity_Prediction, Run_Idle)
{
    PositionVelocityInference inferrer;
    EXPECT_NO_THROW(inferrer.run_idle());
    arwain::system_mode = arwain::OperatingMode::Terminate;
    inferrer.join();
}

#if USE_NCS2
/** \brief Before core_setup, the socket pointers context and responder should be null.
 * After core_setup, they should be non-null (but we don't know exactly what they should be).
 */
TEST(Velocity_Prediction, Core_Setup)
{
    // arwain::config.no_inference = true;
    // PositionVelocityInference inferrer;
    // EXPECT_EQ(inferrer.context, nullptr);
    // EXPECT_EQ(inferrer.responder, nullptr);
    // inferrer.core_setup();
    // EXPECT_NE(inferrer.context, nullptr);
    // EXPECT_NE(inferrer.responder, nullptr);
    FAIL();
}
#else
/** \brief This test not yet implemented for the tensorflow case. */
TEST(Velocity_Prediction, Core_Setup)
{
    FAIL();
}
#endif

/** \brief The call to setup_inference should open log files and populate them with headers.
 * Before calling, position and velocity should be zero-vectors.
 * TODO This test can potentially be improved by making sure the content of the logs
 * is as expected, before deleting them.
 */
TEST(Velocity_Prediction, Setup_Inference)
{
    arwain::config.no_inference = true;
    arwain::folder_date_string = ".";
    PositionVelocityInference inferrer;
    inferrer.position = {1, 1, 1};
    inferrer.velocity = {1, 1, 1};

    EXPECT_NE(inferrer.position, (Vector3{0, 0, 0}));
    EXPECT_NE(inferrer.velocity, (Vector3{0, 0, 0}));

    inferrer.setup_inference();
    EXPECT_TRUE(inferrer.velocity_file.is_open());
    EXPECT_TRUE(inferrer.position_file.is_open());
    
    EXPECT_EQ(inferrer.position, (Vector3{0, 0, 0}));
    EXPECT_EQ(inferrer.velocity, (Vector3{0, 0, 0}));

    inferrer.velocity_file.close();
    inferrer.position_file.close();

    EXPECT_TRUE(std::filesystem::exists("./velocity.txt"));
    EXPECT_TRUE(std::filesystem::exists("./position.txt"));

    EXPECT_GT(std::filesystem::file_size("./velocity.txt"), 0);
    EXPECT_GT(std::filesystem::file_size("./position.txt"), 0);

    std::filesystem::remove("./velocity.txt");
    std::filesystem::remove("./position.txt");
}

/** \brief TODO: I couldn't find a way to stub/mock run_inference without
 * significant changes to the code so this doesn't work for inference mode.
 * No idea what to do about it.
 */
TEST(Velocity_Prediction, Run)
{
    arwain::config.no_inference = true;
    PositionVelocityInference inferrer;

    // If system mode is terminate, run() should return immediately.
    arwain::system_mode = arwain::OperatingMode::Terminate;
    EXPECT_NO_THROW(inferrer.run());

    // If system mode is anything else, run() should loop inside run_idle()
    // and return after the mode is set to Terminate.
    arwain::system_mode = arwain::OperatingMode::SelfTest;
    std::thread offthread = std::thread{
        [this]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds{1000});
            arwain::system_mode = arwain::OperatingMode::Terminate;
        }
    };
    EXPECT_NO_THROW(inferrer.run());
    
    offthread.join();
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
 * fail to find the script, but we can't test that effectively atm.
 */
TEST(NCS2Inferrer, Py_Inference)
{
    // The script is called indirectly by the inferrer constructor.
    arwain::config.no_inference = false;
    EXPECT_NO_THROW(
        NCS2Inferrer inferrer;
        inferrer.ncs2_thread.join();
    );

    arwain::config.no_inference = true;
    EXPECT_NO_THROW(
        NCS2Inferrer inferrer;
    );
}
#endif