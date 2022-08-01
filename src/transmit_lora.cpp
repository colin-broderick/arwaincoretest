#include <thread>
#include <chrono>
#include <bitset>

#include "transmit_lora.hpp"
#include "vector3.hpp"
#include "logger.hpp"
#include "arwain.hpp"
#include "lora.hpp"
#include "timers.hpp"

#define DEBUG_TRANSMIT_LORA 0

std::ostream& operator<<(std::ostream& stream, arwain::PosePacket packet)
{
    stream << "Pose Packet: " << packet.x << " " << packet.y << " " << packet.z << " " << packet.alerts;
    return stream;
}

#if USE_UUBLA == 1
std::ostream& operator<<(std::ostream& stream, arwain::BeaconPacket packet)
{
    stream << "Beacon Packet: " << packet.x << " " << packet.y << " " << packet.z;
    return stream;
}
#endif

/** \brief Transmit or queue a LoRa message when a new UUBLA node joins the network. */
void inform_new_uubla_node(const std::string& node_name)
{
    std::cout << "TODO Not implemented: " << __FUNCTION__ << "\n";
}

/** \brief Transmit or queue a LoRa message when an UUBLA node leaves the network. */
void inform_remove_uubla_node(const std::string& node_name)
{
    std::cout << "TODO Not implemented: " << __FUNCTION__ << "\n";
}

auto get_next_time_slot(int node_id)
{
    // Wait until this node's transmission time slot.
    auto slot_time = std::chrono::high_resolution_clock::now();
    auto ms_count = std::chrono::duration_cast<std::chrono::milliseconds>(
    slot_time.time_since_epoch()).count();
    auto extra = ms_count % 2000;
    ms_count = ms_count + 2000 - extra;
    std::chrono::time_point<std::chrono::high_resolution_clock> tp{std::chrono::milliseconds{ms_count}};
    return tp;
}

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
        true,
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

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::Inference:
            {
                // Open file handles for data logging.
                arwain::Logger lora_file;
                lora_file.open(arwain::folder_date_string + "/lora_log.txt");
                lora_file << "time x y z alerts" << "\n";
                // Set up timing.
                auto time = std::chrono::system_clock::now();
                std::chrono::milliseconds interval{arwain::Intervals::LORA_TRANSMISSION_INTERVAL};

                std::this_thread::sleep_until(get_next_time_slot(arwain::config.node_id));

                while (arwain::system_mode == arwain::OperatingMode::Inference)
                {
                    {
                        arwain::Timers::ScopedTimer t{__FUNCTION__};
                        arwain::PosePacket message;
                        message.metadata = arwain::config.node_id;

                        // std::cout << "Transmission sent: ";
                        // std::cout << std::chrono::high_resolution_clock::now().time_since_epoch().count() << "\n";

                        auto position = arwain::Buffers::POSITION_BUFFER.back();
                        position.z = arwain::Buffers::PRESSURE_BUFFER.back().z;

                        message.x = position.x * 100;
                        message.y = position.y * 100;
                        message.z = position.z * 100;

                        // Read the activity metric, and convert to small int, capping value at 7.
                        double act = arwain::activity_metric.read();
                        message.other = static_cast<uint8_t>(act >= 7 ? 7 : act);

                        // Create alerts flags.
                        message.alerts = arwain::status.falling |
                                        (arwain::status.entangled << 1) |
                                        (arwain::status.attitude << 2) |
                                        (arwain::status.current_stance << 3);

                        // Send transmission.
                        lora.send_message((uint8_t*)&message, arwain::BufferSizes::LORA_MESSAGE_LENGTH);

                        // Log message to file.
                        lora_file << time.time_since_epoch().count() << " " << message << "\n";

                        // Wait until next tick
                        time = time + interval;

                        // Watch for receive until the next scheduled transmission.
                        // int timeout_ms = (time - std::chrono::system_clock::now()).count() / 1000000;
                        // auto [rxd, rxd_message] = lora.receive_string(timeout_ms);
                        // if (rxd)
                        // {
                        //     std::cout << "RECEIVED: " << rxd_message << std::endl;
                        //     if (rxd_message == "C.INFERENCE")
                        //     {
                        //         // arwain::system_mode = arwain::OperatingMode::Inference;
                        //     }
                        //     else if (rxd_message == "C.AUTOCAL")
                        //     {
                        //         // arwain::system_mode = arwain::OperatingMode::AutoCalibration;
                        //     }
                        //     else if (rxd_message == "C.TERMINATE")
                        //     {
                        //         // arwain::system_mode = arwain::OperatingMode::Terminate;
                        //     }
                        //     else if (rxd_message == "C.SELFTEST")
                        //     {
                        //         // arwain::system_mode = arwain::OperatingMode::SelfTest;
                        //     }
                        // }
                    }
                    std::this_thread::sleep_until(get_next_time_slot(arwain::config.node_id));
                }
                
                break;
            }
            default:
            {
                sleep_ms(10);
                break;
            }
        }
    }
}
