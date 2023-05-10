#include <thread>
#include <chrono>
#include <bitset>

#include "arwain/thread.hpp"
#include "arwain/exceptions.hpp"
#include "arwain/logger.hpp"
#include "arwain/transmit_lora.hpp"
#include "arwain/arwain.hpp"

#include "vector3.hpp"
#include "lora.hpp"
#include "timers.hpp"

#if USE_UUBLA
#include "uwb_reader.hpp"
#endif

#if USE_UUBLA
namespace UUBLAState
{
    UUBLA::AutoQueue<uint8_t> new_nodes;
    UUBLA::AutoQueue<uint8_t> dropped_nodes;
    UUBLA::AutoQueue<uint8_t> nearby_nodes;
}
#endif

namespace arwain {
std::ostream& operator<<(std::ostream& stream, arwain::PosePacket packet)
{
    stream << "Pose Packet: " << packet.x << " " << packet.y << " " << packet.z << " " << packet.alerts;
    return stream;
}}

StatusReporting::StatusReporting()
{
    init();
}

/** \brief Computes the next slot in the LoRa schedule where this node is allowed to transmit LoRa messages. */
std::chrono::time_point<std::chrono::high_resolution_clock> StatusReporting::get_next_time_slot(int node_id)
{
    auto node_window_sep = std::chrono::milliseconds{300};
    auto window_size = std::chrono::milliseconds{1000};
    auto offset = (node_id - 1) * node_window_sep;
    auto slot_time = std::chrono::high_resolution_clock::now();
    auto sec_count = std::chrono::duration_cast<std::chrono::seconds>(slot_time.time_since_epoch());
    return std::chrono::time_point<std::chrono::high_resolution_clock>{sec_count + std::chrono::seconds{1} + offset};
}

void StatusReporting::core_setup()
{
    lora = LoRa<SPIDEVICEDRIVER>{
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
}

void StatusReporting::setup_inference()
{
    lora_file.open(arwain::folder_date_string + "/lora_log.txt");
    lora_file << "time x y z alerts" << "\n";
}

void StatusReporting::cleanup_inference()
{
    lora_file.close();
}

void StatusReporting::run_idle()
{
    sleep_ms(10);
}

bool StatusReporting::set_stance_detection_pointer(StanceDetection& stance)
{
    // TODO Should I delete the pointer first?
    this->stance_detection_handle = &stance;
    return true;
}

bool StatusReporting::set_uubla_wrapper_handle(UublaWrapper& uubla)
{
    // TODO Should I delete the pointer first?
    this->uubla_wrapper_handle = &uubla;
    return true;
}

void StatusReporting::run_inference()
{
    setup_inference();

    std::this_thread::sleep_until(get_next_time_slot(arwain::config.node_id));

    while (mode == arwain::OperatingMode::Inference)
    {
        if (stance_detection_handle == nullptr)
        {
            // TODO return arwain code for early exit
            throw std::runtime_error{"You haven't set a stance detection pointer."};
        }

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
        pose_message.alerts = stance_detection_handle->get_falling_state()
                            | (stance_detection_handle->get_entangled_state() << 1)
                            | (stance_detection_handle->get_attitude() << 2)
                            | (stance_detection_handle->get_stance() << 3);

        #if USE_UUBLA
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
            pose_message.partner_distance = static_cast<int8_t>(uubla_wrapper_handle->get_distance(0) * 2.0);
            std::cout << "Partner distance f = " << uubla_wrapper_handle->get_distance(0) << "\n";
            std::cout << "Partner distance i = " << static_cast<int>(pose_message.partner_distance) << "\n";
            std::cout << "\n";
        }
        #endif
            
        // Send transmission.
        lora.send_message((uint8_t*)&pose_message, arwain::BufferSizes::LORA_MESSAGE_LENGTH + 1);

        // Log pose_message to file.
        lora_file << std::chrono::high_resolution_clock::now().time_since_epoch().count() << " " << pose_message << "\n";

        std::this_thread::sleep_until(get_next_time_slot(arwain::config.node_id));
    }

    cleanup_inference();
}

void StatusReporting::run()
{
    if (arwain::config.no_lora)
    {
        return;
    }
    
    while (mode != arwain::OperatingMode::Terminate)
    {
        switch (mode)
        {
            case arwain::OperatingMode::Inference:
                run_inference();
                break;
            default:
                run_idle();
                break;
        }
    }
}

bool StatusReporting::init()
{
    core_setup();
    if (job_thread.joinable())
    {
        job_thread.join();
    }
    job_thread = ArwainThread{&StatusReporting::run, "arwain_stat_th", this};
    return true;
}

bool StatusReporting::join()
{
    while (!job_thread.joinable())
    {
        sleep_ms(1);
    }
    if (job_thread.joinable())
    {
        job_thread.join();
        return true;
    }
    return false;
}

#if USE_UUBLA
std::ostream& operator<<(std::ostream& stream, arwain::BeaconPacket packet)
{
    stream << (int)packet.beacon_id << " " << (int)packet.arwain_node_id;
    return stream;
}
#endif

#if USE_UUBLA
/** \brief Transmit or queue a LoRa message when a new UUBLA node joins the network. */
void inform_new_uubla_node(const std::string& node_name)
{
    UUBLAState::new_nodes.add(std::stoi(node_name));
}
#endif

#if USE_UUBLA
/** \brief Transmit or queue a LoRa message when an UUBLA node leaves the network. */
void inform_remove_uubla_node(const std::string& node_name)
{
    std::cout << "TODO Not implemented: " << __FUNCTION__ << "\n";
    UUBLAState::dropped_nodes.add(std::stoi(node_name));
}
#endif
