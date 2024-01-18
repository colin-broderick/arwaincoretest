#include <gtest/gtest.h>

#include <arwain/event_manager.hpp>

#include "arwain/altimeter.hpp"
#include "arwain/arwain.hpp"
#include "arwain/events.hpp"
#include "test_base.hpp"

/** \brief Test that the init() function in Altimeter correctly executes the true path. */
HARDWARE_TEST(Altimeter, init__success)
{
    // First create the altimeter in an uninitalized state.
    arwain::config.no_pressure = true;
    Altimeter altimeter;

    // Now initialize it.
    arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);

    arwain::config.no_pressure = false;
    EXPECT_TRUE(altimeter.init());

    // And join it before quitting.
    EXPECT_TRUE(altimeter.join());
}

/** \brief If pressure is disabled, init returns false and the rest of the
 * class is uninitialized.
 */
HARDWARE_TEST(Altimeter, init__fail)
{
    arwain::config.no_pressure = true;
    Altimeter altimeter;
    EXPECT_TRUE(altimeter.init());
    EXPECT_TRUE(altimeter.job_thread.joinable());
    EXPECT_TRUE(altimeter.join());
}

/** \brief Test that alimeter.join returns the expected value depending on system state. */
HARDWARE_TEST(Altimeter, join)
{
    {
        arwain::config.no_pressure = true;
        Altimeter altimeter;
        EXPECT_TRUE(altimeter.init());
        EXPECT_TRUE(altimeter.job_thread.joinable());
        EXPECT_TRUE(altimeter.join());
    }

    {
        arwain::config.no_pressure = false;
        Altimeter altimeter;
        EXPECT_TRUE(altimeter.job_thread.joinable());
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);

        EXPECT_TRUE(altimeter.join());
    }
}

HARDWARE_TEST(Altimeter, Altimeter)
{
    EXPECT_NO_THROW(
        Altimeter alt;
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
        alt.join();
    );
}

/** \brief Hit each branch of the run method. */
HARDWARE_TEST(Altimeter, run)
{
    // By creating an altimeter and switching to each of various modes, we run each branch
    // of the run() method.
    EXPECT_NO_THROW(
        arwain::config.no_pressure = false;
        Altimeter altimeter;
        sleep_ms(500); // Currently executing Idle loop.
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Inference);
        sleep_ms(500); // Currently executing inference loop.
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::TestSerial); // Just a random mode for the default branch.
        sleep_ms(500);
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
        EXPECT_TRUE(altimeter.join());
    );
}

/** \brief Test achieves coverage of core_setup function but needs to check state. */
HARDWARE_TEST(Altimeter, core_setup)
{
    arwain::config.no_pressure = true;
    Altimeter alt;
    EXPECT_NO_THROW(alt.core_setup());
    alt.join();
}

HARDWARE_TEST(Altimeter, run_inference)
{
    // Should go straight into the inference loop and then terminate after a delay.
    arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Inference);
    arwain::config.no_pressure = false;
    Altimeter altimeter;
    sleep_ms(10);
    arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    EXPECT_TRUE(altimeter.join());
}

/** \brief Check the run_idle function runs until mode tells it to stop. */
HARDWARE_TEST(Altimeter, run_idle)
{
    // Create an initialized altimeter.
    arwain::config.no_pressure = false;
    Altimeter altimeter;

    // Join it immediately to stop the job thread.
    arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    altimeter.join();

    // Start a thread that will change the system mode to terminate after specified time,
    arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Idle);
    std::jthread mode_thread = std::jthread{
        []()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds{1000});
            arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
        }
    };

    // And run the idle loop until it quits.
    EXPECT_NO_THROW(altimeter.run_idle());
}

/** \brief Opens a log file and puts a header into it. */
HARDWARE_TEST(Altimeter, setup_inference)
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
    altimeter.join();
}

HARDWARE_TEST(Altimeter, cleanup_inference)
{
    arwain::config.no_pressure = true;
    Altimeter altimeter;
    altimeter.pressure_log.open("/dev/null");
    EXPECT_TRUE(altimeter.pressure_log.is_open());
    altimeter.cleanup_inference();
    EXPECT_FALSE(altimeter.pressure_log.is_open());
    altimeter.join();
}
