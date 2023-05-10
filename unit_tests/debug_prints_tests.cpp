#include <gtest/gtest.h>

#include "arwain/std_output.hpp"
#include "arwain/arwain.hpp"

TEST(DebugPrints, run)
{
    {
        EventManager::switch_mode_event.invoke(arwain::OperatingMode::Idle);
        DebugPrints debug;
        EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
        debug.join();
    }
    {
        EventManager::switch_mode_event.invoke(arwain::OperatingMode::Idle);
        arwain::config.log_to_stdout = true;
        DebugPrints debug;
        EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
        debug.join();
    }
    {
        EventManager::switch_mode_event.invoke(arwain::OperatingMode::Inference);
        arwain::config.log_to_stdout = true;
        DebugPrints debug;
        sleep_ms(200);
        EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
        debug.join();
    }
    {
        EventManager::switch_mode_event.invoke(arwain::OperatingMode::Inference);
        arwain::config.log_to_stdout = true;
        StanceDetection stance;
        DebugPrints debug;
        debug.set_stance_detection_pointer(stance);
        sleep_ms(200);
        EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
        debug.join();
        stance.join();
    }
}
