#include <gtest/gtest.h>

#include "arwain/arwain.hpp"
#include "arwain/transmit_lora.hpp"
#include "arwain/events.hpp"

#include <arwain/devices/rfm95w.hpp>

TEST(StatusReporting, setup_inference)
{
    EXPECT_FALSE(std::filesystem::exists("./lora_log.txt"));
    arwain::config.no_lora = true;
    arwain::folder_date_string = ".";
    StatusReporting status;
    status.setup_inference();
    EXPECT_TRUE(status.lora_file.is_open());
    status.cleanup_inference();
    EXPECT_FALSE(status.lora_file.is_open());
    EXPECT_TRUE(std::filesystem::exists("./lora_log.txt"));
    std::filesystem::remove("./lora_log.txt");
    EXPECT_FALSE(std::filesystem::exists("./lora_log.txt"));
    arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    status.join();
}

TEST(StatusReporting, run_idle)
{
    arwain::config.no_lora = true;
    StatusReporting status;
    EXPECT_NO_THROW(status.run_idle());
    status.join();
}

TEST(StatusReporting, set_stance_detection_pointer)
{
    arwain::config.no_lora = true;
    StanceDetection stance;
    StatusReporting status;
    EXPECT_EQ(status.stance_detection_handle, nullptr);
    EXPECT_NO_THROW(status.set_stance_detection_pointer(stance));
    EXPECT_NE(status.stance_detection_handle, nullptr);
    arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    stance.join();
    status.join();
}

TEST(StatusReporting, run_inference)
{
    arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Inference);
    arwain::config.no_lora = false;
    StanceDetection stance;
    StatusReporting status;
    status.set_stance_detection_pointer(stance);
    std::thread th = std::thread{[](){
        sleep_ms(3000);
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    }};
    status.join();
    stance.join();
    th.join();
    std::filesystem::remove("./lora_log.txt");
}
