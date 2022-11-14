#include <thread>
#include <chrono>
#include <bitset>

#include "transmit_lora.hpp"
#include "vector3.hpp"
#include "logger.hpp"
#include "arwain.hpp"
#include "lora.hpp"
#include "timers.hpp"
#if USE_UUBLA == 1
#include "uubla.hpp"
#endif

#if USE_UUBLA == 1
namespace UUBLAState
{
    UUBLA::AutoQueue<uint8_t> new_nodes;
    UUBLA::AutoQueue<uint8_t> dropped_nodes;
    UUBLA::AutoQueue<uint8_t> nearby_nodes;
}
#endif

std::ostream& operator<<(std::ostream& stream, arwain::PosePacket packet)
{
    stream << "Pose Packet: " << packet.x << " " << packet.y << " " << packet.z << " " << packet.alerts;
    return stream;
}

#if USE_UUBLA == 1
std::ostream& operator<<(std::ostream& stream, arwain::BeaconPacket packet)
{
    stream << (int)packet.beacon_id << " " << (int)packet.arwain_node_id;
    return stream;
}
#endif

#if USE_UUBLA == 1
/** \brief Transmit or queue a LoRa message when a new UUBLA node joins the network. */
void inform_new_uubla_node(const std::string& node_name)
{
    UUBLAState::new_nodes.add(std::stoi(node_name));
}
#endif

#if USE_UUBLA == 1
/** \brief Transmit or queue a LoRa message when an UUBLA node leaves the network. */
void inform_remove_uubla_node(const std::string& node_name)
{
    std::cout << "TODO Not implemented: " << __FUNCTION__ << "\n";
    UUBLAState::dropped_nodes.add(std::stoi(node_name));
}
#endif

/** \brief Computes the next slot in the LoRa schedule where this node is allowed to transmit LoRa messages. */
std::chrono::time_point<std::chrono::high_resolution_clock> get_next_time_slot(int node_id)
{
    auto node_window_sep = std::chrono::milliseconds{300};
    auto window_size = std::chrono::milliseconds{1000};
    auto offset = (node_id - 1) * node_window_sep;
    auto slot_time = std::chrono::high_resolution_clock::now();
    auto sec_count = std::chrono::duration_cast<std::chrono::seconds>(slot_time.time_since_epoch());
    return std::chrono::time_point<std::chrono::high_resolution_clock>{sec_count + std::chrono::seconds{1} + offset};
}

/** \brief Forms and transmits LoRa messages on a loop. */
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

    // TODO Put these configruations in the LoRa constructor and init.
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
                    arwain::PosePacket pose_message;
                    pose_message.metadata = arwain::config.node_id;

                    auto position = arwain::Buffers::POSITION_BUFFER.back();
                    position.z = arwain::Buffers::PRESSURE_BUFFER.back().z;

                    pose_message.x = position.x * 100;
                    pose_message.y = position.y * 100;
                    pose_message.z = position.z * 100;

                    // Read the activity metric, and convert to small int, capping value at 7.
                    double act = arwain::activity_metric.read();
                    pose_message.other = static_cast<uint8_t>(act >= 7 ? 7 : act);

                    // Create alerts flags.
                    pose_message.alerts = StanceDetection::get_falling_state()
                                          | (StanceDetection::get_entangled_state() << 1)
                                          | (StanceDetection::get_attitude() << 2)
                                          | (StanceDetection::get_stance() << 3);

                    #if USE_UUBLA == 1
                    // As written, only one piece of beacon info can be sent at a time.
                    // This is vulnerable to failure since it cannot be guaranteed that any
                    // given LoRa message is actually received.
                    if (UUBLAState::new_nodes.size() != 0)
                    {
                        pose_message.add_beacon(UUBLAState::new_nodes.next());
                    }
                    else if (UUBLAState::dropped_nodes.size() != 0)
                    {
                        pose_message.drop_beacon(UUBLAState::dropped_nodes.next());
                    }
                    else if (UUBLAState::nearby_nodes.size() != 0)
                    {
                        pose_message.nearby_beacon(UUBLAState::nearby_nodes.next());
                    }
                    if (arwain::config.node_id == 2)
                    {
                        pose_message.partner_distance = static_cast<int8_t>(arwain::uubla_handle->get_distance(0) * 2.0);
                        std::cout << "Partner distance f = " << arwain::uubla_handle->get_distance(0) << "\n";
                        std::cout << "Partner distance i = " << static_cast<int>(pose_message.partner_distance) << "\n";
                        std::cout << "\n";
                    }
                    #endif
                    
                    // Send transmission.
                    lora.send_message((uint8_t*)&pose_message, arwain::BufferSizes::LORA_MESSAGE_LENGTH + 1);

                    // Log pose_message to file.
                    lora_file << std::chrono::high_resolution_clock::now().time_since_epoch().count() << " " << pose_message << "\n";

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
