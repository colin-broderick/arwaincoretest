#include <gtest/gtest.h>

#include "altimeter.hpp"
#include "arwain.hpp"

/** \brief Needs hardware*/
TEST(HARDWARE_Altimeter, Init_Success)
{
    FAIL(); 
}

/** \brief If pressure is disabled, init returns false and the rest of the
 * class is uninitialized.
 */
TEST(Altimeter, Init_Failure)
{
    arwain::config.no_pressure = true;
    Altimeter altimeter;
    EXPECT_FALSE(altimeter.init());
    EXPECT_FALSE(altimeter.job_thread.joinable());
}

/** \brief Can't currently test active join without hardware. */
TEST(HARDWARE_Altimeter, Join)
{
    {
        arwain::config.no_pressure = true;
        Altimeter altimeter;
        EXPECT_FALSE(altimeter.init());
        EXPECT_FALSE(altimeter.job_thread.joinable());
        altimeter.join();
    }
    FAIL();
}

/** \brief Needs hardware. */
TEST(HARDWARE_Altimeter, Constructor)
{
    FAIL(); 
}

/** \brief Needs hardware. */
TEST(HARDWARE_Altimeter, Run)
{
    FAIL(); 
}

/** \brief Needs hardware. */
TEST(HARDWARE_Altimeter, Core_Setup)
{
    FAIL(); 
}

/** \brief Needs hardware. */
TEST(HARDWARE_Altimeter, Run_Inference)
{
    FAIL(); 
}

/** \brief EVEN THIS needs hardware. */
TEST(HARDWARE_Altimeter, Run_Idle)
{
    FAIL(); 
}

/** \brief Opens a log file and puts a header into it. */
TEST(Altimeter, Setup_Inference)
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

TEST(Altimeter, Cleanup_Inference)
{
    arwain::config.no_pressure = true;
    Altimeter altimeter;
    altimeter.pressure_log.open("/dev/null");
    EXPECT_TRUE(altimeter.pressure_log.is_open());
    altimeter.cleanup_inference();
    EXPECT_FALSE(altimeter.pressure_log.is_open());
}
