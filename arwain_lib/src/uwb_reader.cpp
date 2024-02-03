#include <tuple>
#include <string>
#include <sstream>
#include <algorithm>

#include "arwain/arwain.hpp"
#include "arwain/serial.hpp"
#include "arwain/logger.hpp"
#include "arwain/exceptions.hpp"
#include "arwain/transmit_lora.hpp"
#include "arwain/uwb_reader.hpp"
#include "arwain/service_manager.hpp"
#include "arwain/hybrid_positioner.hpp"

#include "arwain/websocket.hpp"
#include "arwain/ws_includes.hpp"

#include "uubla/utils.hpp"
#include "uubla/events.hpp"

UUBLA::Network* l_uubla = nullptr;

void position_callback(int nodeID, double x, double y, double z)
{
    if (l_uubla == nullptr)
    {
        return;
    }

    std::string str_node_id = UUBLA::Node::name_from_int(nodeID);

    if (l_uubla->get_nodes().count(str_node_id) == 0)
    {
        return;
    }

    UUBLA::Node& node = l_uubla->get_nodes().at(str_node_id);
    node.fix_position();
    node.set_position({x, y, z});
}

#define PUBLISH_HYBRID_POSITIONS

void publish_positions(arwain::WebSocketServer& server, UUBLA::Network& uubla)
{
    Message<PositionData> message;

#ifdef PUBLISH_UUBLA_POSITIONS
    auto data = uubla.get_nodes();
    for (auto& [k, v] : data)
    {
        Vector3 pos = v.get_position();
        message.set_header({v.id(), MessageType::position, {123, 123}});
        message.set_data({pos.x, pos.y, pos.z, v.position_fixed() ? 1 : 0});
        // server.add_history(message);
        server.send_message(message.to_string());
    }

#elif defined(PUBLISH_HYBRID_POSITIONS)
    auto hyb = ServiceManager::get_service<HybridPositioner>("HybridPositioner");
    if (hyb)
    {
        auto pos = hyb->get_position();

        message.set_header({2, MessageType::position, {123, 123}});
        message.set_data({pos.x, pos.z, pos.y, 0});
        server.send_message(message.to_string());
    }

#endif
}

void state_messages_callback(arwain::WebSocketServer* ws, std::shared_ptr<WssServer::Connection> cn, std::string& msg)
{
    Header header;
    header.deserialise(msg);

    switch (header.type)
    {
        case MessageType::history_request:
            // ws->send_history();
            break;
        case MessageType::position:
        {
            Message<PositionData> position_message;
            position_message.deserialize(msg);
            position_callback(
                position_message.get_header().node_id,
                position_message.get_data().X,
                position_message.get_data().Y,
                position_message.get_data().Z);
            break;
        }
        case MessageType::rotation:
            // TODO
            break;
        case MessageType::stance:
            // TODO
            break;
    }
}

std::unique_ptr<arwain::WebSocketServer> server;

UublaWrapper::UublaWrapper()
{
    ServiceManager::register_service(this, service_name);

    // The websocket server exists to receive position data from the Unity front end.
    // Upon reception, data is fed to the UUBLA instance running here and there it is
    // used for solving for tag position.
    server = std::make_unique<arwain::WebSocketServer>(8081, state_messages_callback);
    init();
}

UublaWrapper::~UublaWrapper()
{
    UUBLA::run_flag = false;
    ServiceManager::unregister_service(service_name);
}

void UublaWrapper::core_setup()
{
    m_uubla = std::make_unique<UUBLA::Network>();
    l_uubla = m_uubla.get();
}

void UublaWrapper::run_idle()
{
    sleep_ms(10);
}

void UublaWrapper::run()
{
    // TODO Currently configured to act as an UUBLA master; need to generalize.
    if (!arwain::config.use_uwb_positioning || arwain::config.node_id != 2)
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

void UublaWrapper::setup_inference()
{
    m_uubla->force_plane(true);
    m_uubla->set_ewma_gain(0.1);
    // m_uubla->add_node_callback = inform_new_uubla_node; // Replaced with event registrations.
    // m_uubla->remove_node_callback = inform_remove_uubla_node; // Replaced with event registrations.
    serial_reader_th = std::jthread{serial_reader_fn, m_uubla.get(), arwain::config.uubla_serial_port, 115200};
}

std::atomic<bool> runflag = true;

bool UublaWrapper::cleanup_inference()
{
    UUBLA::run_flag = false;
    // serial_reader_th.request_stop();
    return true;
}

void UublaWrapper::run_inference()
{
    setup_inference();

    // TODO: Get the framerate (30) from somewhere sensible, or hard code?
    IntervalTimer<std::chrono::milliseconds> timer{1000 / 30};

    auto last_count = timer.count();
    while (mode == arwain::OperatingMode::Inference)
    {
        m_uubla->process_queue();
        m_uubla->solve_map();
        m_uubla->process_callbacks();
        publish_positions(*server, *m_uubla);
        auto now_count = timer.count();
        arwain::Events::new_uwb_position_event.invoke({get_own_position(), (now_count - last_count) / 1000.0});
        last_count = now_count;
        timer.await();
    }

    cleanup_inference();
}

bool UublaWrapper::init()
{
    core_setup();
    job_thread = std::jthread{std::bind_front(&UublaWrapper::run, this)};
    return true;
}

bool UublaWrapper::join()
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

double UublaWrapper::get_distance(const int position)
{
    // TODO Do we still need this function?  Library doesn't support it.
    // return m_uubla->get_distance(position);
    return 0;
}

Vector3 UublaWrapper::get_own_position() const
{
    // TODO Remove hard-coded 0002; get from config.
    return m_uubla->get_node_position("0002"); // return get_node_position(own_id); // TODO
}

Vector3 UublaWrapper::get_node_position(const std::string& node_name) const
{
    return m_uubla->get_node_position(node_name);
}
