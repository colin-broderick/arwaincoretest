#include <thread>
#include <chrono>

#include "vector3.hpp"
#include "logger.hpp"
#include "arwain.hpp"

/** \brief Forms and transmits LoRa messages on a loop.
 */
void transmit_lora()
{
    if (arwain::config.no_lora)
    {
        return;
    }

    LoRa lora{
        arwain::config.lora_address,
        false,
        arwain::config.lora_rf_frequency,
        arwain::config.lora_bandwidth,
        arwain::config.lora_spread_factor
    };

    // TODO Put these configruations in the LORa constructor and init.
    // lora.setTXPower(CONFIG.lora_tx_power);
    // lora.setCodingRate(CONFIG.lora_coding_rate);
    // lora.setHeaderMode(CONFIG.lora_header_mode);
    // if (CONFIG.lora_enable_crc)
    // {
        // lora.enableCRC();
    // }
    // lora.setSyncWord(0x12);

    // Local buffers.
    arwain::Logger lora_file;
    vector3 position;

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{arwain::Intervals::LORA_TRANSMISSION_INTERVAL};

    // Open file handles for data logging.
    if (arwain::config.log_to_file)
    {
        lora_file.open(arwain::folder_date_string + "/lora_log.txt");
        lora_file << "time x y z alerts" << "\n";
    }

    while (!arwain::shutdown)
    {
        arwain::LoraPacket message;

        { // Get positions as float16.
            std::lock_guard<std::mutex> lock{arwain::Locks::POSITION_BUFFER_LOCK};
            position = arwain::Buffers::POSITION_BUFFER.back();
        }

        message.x = position.x * 100;
        message.y = position.y * 100;
        message.z = position.z * 100;

        // Create alerts flags.
        message.alerts = arwain::status.falling | (arwain::status.entangled << 1) | (arwain::status.current_stance << 2);

        // Reset critical status flags now they have been read.
        arwain::status.falling = arwain::StanceDetector::NotFalling;
        arwain::status.entangled = arwain::StanceDetector::NotEntangled;

        // Send transmission.
        lora.send_message((uint8_t*)&message, arwain::BufferSizes::LORA_MESSAGE_LENGTH);

        // TODO: Check the log format is as expected and usable.
        if (arwain::config.log_to_file)
        {
            lora_file << time.time_since_epoch().count() << " " << message << "\n";
        }

        // Wait until next tick
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    // Close log file handle.
    if (arwain::config.log_to_file)
    {
        lora_file.close();
    }
}
