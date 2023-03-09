#include <gtest/gtest.h>

#include "altimeter.hpp"
#include "arwain.hpp"

/** \brief Test that the init() function in Altimeter correctly executes the true path. */
TEST(Altimeter, init__success)
{
    // First create the altimeter in an uninitalized state.
    arwain::config.no_pressure = true;
    Altimeter altimeter;

    // Now initialize it.
    arwain::config.no_pressure = false;
    EXPECT_TRUE(altimeter.init());

    // And join it before quitting.
    arwain::system_mode = arwain::OperatingMode::Terminate;
    EXPECT_TRUE(altimeter.join());
}

/** \brief If pressure is disabled, init returns false and the rest of the
 * class is uninitialized.
 */
TEST(Altimeter, init__fail)
{
    arwain::config.no_pressure = true;
    Altimeter altimeter;
    EXPECT_FALSE(altimeter.init());
    EXPECT_FALSE(altimeter.job_thread.joinable());
}

/** \brief Test that alimeter.join returns the expected value depending on system state. */
TEST(Altimeter, join)
{
    FAIL();
    {
        arwain::config.no_pressure = true;
        Altimeter altimeter;
        EXPECT_FALSE(altimeter.init());
        EXPECT_FALSE(altimeter.job_thread.joinable());
        EXPECT_FALSE(altimeter.join());
    }

    {
        arwain::config.no_pressure = false;
        Altimeter altimeter;
        EXPECT_TRUE(altimeter.job_thread.joinable());
        arwain::system_mode = arwain::OperatingMode::Terminate;
        EXPECT_TRUE(altimeter.join());
    }
}

TEST(Altimeter, Altimeter)
{
    EXPECT_NO_THROW(
        Altimeter alt;
        arwain::system_mode = arwain::OperatingMode::Terminate;
        alt.join();
    );
}

/** \brief Hit each branch of the run method. */
TEST(Altimeter, run)
{
    // By creating an altimeter and switching to each of various modes, we run each branch
    // of the run() method.
    EXPECT_NO_THROW(
        arwain::config.no_pressure = false;
        Altimeter altimeter;
        sleep_ms(500); // Currently executing Idle loop.
        arwain::system_mode = arwain::OperatingMode::Inference;
        sleep_ms(500); // Currently executing inference loop.
        arwain::system_mode = arwain::OperatingMode::TestSerial; // Just a random mode for the default branch.
        sleep_ms(500);
        arwain::system_mode = arwain::OperatingMode::Terminate;
        EXPECT_TRUE(altimeter.join());
    );
}

/** \brief Test achieves coverage of core_setup function but needs to check state. */
TEST(Altimeter, core_setup)
{
    arwain::config.no_pressure = true;
    Altimeter alt;
    EXPECT_NO_THROW(alt.core_setup());
}

TEST(Altimeter, run_inference)
{
    // Should go straight into the inference loop and then terminate after a delay.
    arwain::system_mode = arwain::OperatingMode::Inference;
    arwain::config.no_pressure = false;
    Altimeter altimeter;
    sleep_ms(10);
    arwain::system_mode = arwain::OperatingMode::Terminate;
    EXPECT_TRUE(altimeter.join());
}

/** \brief Check the run_idle function runs until mode tells it to stop. */
TEST(Altimeter, run_idle)
{
    // Create an initialized altimeter.
    arwain::config.no_pressure = false;
    Altimeter altimeter;

    // Join it immediately to stop the job thread.
    arwain::system_mode = arwain::OperatingMode::Terminate;
    altimeter.join();

    // Start a thread that will change the system mode to terminate after specified time,
    arwain::system_mode = arwain::OperatingMode::Idle;
    std::thread mode_thread = std::thread{
        []()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds{1000});
            arwain::system_mode = arwain::OperatingMode::Terminate;
        }
    };

    // And run the idle loop until it quits.
    EXPECT_NO_THROW(altimeter.run_idle());
    mode_thread.join();
}

/** \brief Opens a log file and puts a header into it. */
TEST(Altimeter, setup_inference)
{
    arwain::config.no_pressure = true;
    Altimeter altimeter;
    arwain::folder_date_string = ".";
    altimeter.setup_inference();
    EXPECT_TRUE(altimeter.pressure_log.is_open());
    altimeter.pressure_log.close();
    EXPECT_FALSE(altimeter.pressure_log.is_open());
    EXPECT_TRUE(std::filesystem::exists("./pressure.txt"));
    EXPECT_GT(std::filesystem::file_size("./pressure.txt"), 0);
    std::filesystem::remove("./pressure.txt");
    EXPECT_FALSE(std::filesystem::exists("./pressure.txt"));
}

TEST(Altimeter, cleanup_inference)
{
    arwain::config.no_pressure = true;
    Altimeter altimeter;
    altimeter.pressure_log.open("/dev/null");
    EXPECT_TRUE(altimeter.pressure_log.is_open());
    altimeter.cleanup_inference();
    EXPECT_FALSE(altimeter.pressure_log.is_open());
}
