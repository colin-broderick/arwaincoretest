#include <gtest/gtest.h>

#include "arwain.hpp"
#include "lora.hpp"
#include "transmit_lora.hpp"

/** \brief The configure() method is called by the constructor only, and executes
 * various branches depending on the information provided by the constructor.
 * 
 * Configure writes hardware but changes to other state, so can't effectively test
 * the result without hardware. But we can check for code coverage with no crashes.
 */
TEST(LoRa, configure)
{
    // Branch for each frequency.
    for (auto freq : {
            LoRa<MockSpiDevice>::Frequency::FREQ_433,
            LoRa<MockSpiDevice>::Frequency::FREQ_868,
            LoRa<MockSpiDevice>::Frequency::FREQ_915,
            LoRa<MockSpiDevice>::Frequency::FREQ_923
        })
    {
        LoRa<MockSpiDevice> lora{
            "address",
            false,
            freq,
            LoRa<MockSpiDevice>::Bandwidth::BW_250K,
            LoRa<MockSpiDevice>::SpreadFactor::SF_12
        };
        EXPECT_EQ(lora.frequency_mhz, freq);
    }

    // Branch for each bandwidth.
    for (auto bw : {
            LoRa<MockSpiDevice>::Bandwidth::BW_7_8K,
            LoRa<MockSpiDevice>::Bandwidth::BW_10_4K,
            LoRa<MockSpiDevice>::Bandwidth::BW_15_6K,
            LoRa<MockSpiDevice>::Bandwidth::BW_20_8K,
            LoRa<MockSpiDevice>::Bandwidth::BW_31_25K,
            LoRa<MockSpiDevice>::Bandwidth::BW_41_7K,
            LoRa<MockSpiDevice>::Bandwidth::BW_62_5K,
            LoRa<MockSpiDevice>::Bandwidth::BW_125K,
            LoRa<MockSpiDevice>::Bandwidth::BW_250K,
            LoRa<MockSpiDevice>::Bandwidth::BW_500K
        })
    {
        LoRa<MockSpiDevice> lora{
            "address",
            false,
            LoRa<MockSpiDevice>::Frequency::FREQ_868,
            bw,
            LoRa<MockSpiDevice>::SpreadFactor::SF_12
        };
        EXPECT_EQ(lora.bandwidth_khz, bw);
        EXPECT_FALSE(lora.is_receiver);
    }

    // Branch for each spreading factor.
    // Also set receiver=true here to hit that branch.
    for (auto sf : {
            LoRa<MockSpiDevice>::SpreadFactor::SF_6,
            LoRa<MockSpiDevice>::SpreadFactor::SF_7,
            LoRa<MockSpiDevice>::SpreadFactor::SF_8,
            LoRa<MockSpiDevice>::SpreadFactor::SF_9,
            LoRa<MockSpiDevice>::SpreadFactor::SF_10,
            LoRa<MockSpiDevice>::SpreadFactor::SF_11,
            LoRa<MockSpiDevice>::SpreadFactor::SF_12
        })
    {
        LoRa<MockSpiDevice> lora{
            "address",
            true,
            LoRa<MockSpiDevice>::Frequency::FREQ_868,
            LoRa<MockSpiDevice>::Bandwidth::BW_500K,
            sf
        };
        EXPECT_EQ(lora.spread_factor, sf);
        EXPECT_TRUE(lora.is_receiver);
    }
}

// Can't currently do this effectively because we can't
// set IRQFlag or change IRQ_TXDONE within send_message.
TEST(LoRa, send_message)
{
    // SUCCEED();
    LoRa<MockSpiDevice> lora{"address", true};
    std::string message = "hello";
    lora.send_message(message);
}

TEST(LoRa, test_chip)
{
    // In the test setup this can only return 0.
    LoRa<MockSpiDevice> lora{"address", true};
    EXPECT_EQ(lora.test_chip(), 0);
}

TEST(LoRa, read_fifo)
{
    LoRa<MockSpiDevice> lora{"address", true};
    uint8_t buffer[10] = {0};
    lora.read_fifo(8, buffer);
    EXPECT_EQ(buffer[3], 0xCC);
}

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
    arwain::system_mode = arwain::OperatingMode::Terminate;
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
    arwain::system_mode = arwain::OperatingMode::Terminate;
    stance.join();
    status.join();
}

TEST(StatusReporting, run_inference)
{
    arwain::system_mode = arwain::OperatingMode::Inference;
    arwain::config.no_lora = false;
    StanceDetection stance;
    StatusReporting status;
    status.set_stance_detection_pointer(stance);
    std::thread th = std::thread{[](){
        sleep_ms(3000);
        arwain::system_mode = arwain::OperatingMode::Terminate;
    }};
    status.join();
    stance.join();
    th.join();
    std::filesystem::remove("./lora_log.txt");
}
