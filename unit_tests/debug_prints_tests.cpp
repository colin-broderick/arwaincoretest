#include <gtest/gtest.h>

#include "arwain/std_output.hpp"
#include "arwain/arwain.hpp"
#include "arwain/events.hpp"

TEST(DebugPrints, run)
{
    {
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Idle);
        DebugPrints debug;
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
        debug.join();
    }
    {
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Idle);
        arwain::config.log_to_stdout = true;
        DebugPrints debug;
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
        debug.join();
    }
    {
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Inference);
        arwain::config.log_to_stdout = true;
        DebugPrints debug;
        sleep_ms(200);
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
        debug.join();
    }
    {
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Inference);
        arwain::config.log_to_stdout = true;
        StanceDetection stance;
        DebugPrints debug;
        debug.set_stance_detection_pointer(stance);
        sleep_ms(200);
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
        debug.join();
        stance.join();
    }
}
