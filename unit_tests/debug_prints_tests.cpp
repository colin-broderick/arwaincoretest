#include <gtest/gtest.h>

#include "std_output.hpp"
#include "arwain.hpp"

TEST(DebugPrints, run)
{
    {
        arwain::system_mode = arwain::OperatingMode::Idle;
        DebugPrints debug;
        arwain::system_mode = arwain::OperatingMode::Terminate;
        debug.join();
    }
    {
        arwain::system_mode = arwain::OperatingMode::Idle;
        arwain::config.log_to_stdout = true;
        DebugPrints debug;
        arwain::system_mode = arwain::OperatingMode::Terminate;
        debug.join();
    }
    {
        arwain::system_mode = arwain::OperatingMode::Inference;
        arwain::config.log_to_stdout = true;
        DebugPrints debug;
        sleep_ms(200);
        arwain::system_mode = arwain::OperatingMode::Terminate;
        debug.join();
    }
    {
        arwain::system_mode = arwain::OperatingMode::Inference;
        arwain::config.log_to_stdout = true;
        StanceDetection stance;
        DebugPrints debug;
        debug.set_stance_detection_pointer(stance);
        sleep_ms(200);
        arwain::system_mode = arwain::OperatingMode::Terminate;
        debug.join();
        stance.join();
    }
}
