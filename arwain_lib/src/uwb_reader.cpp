#include <tuple>
#include <string>
#include <sstream>
#include <algorithm>
#include <thread>

#include "arwain/arwain.hpp"
#include "arwain/serial.hpp"
#include "arwain/logger.hpp"
#include "arwain/exceptions.hpp"
#include "arwain/transmit_lora.hpp"
#include "arwain/uwb_reader.hpp"
#include "arwain/service_manager.hpp"
#include "arwain/hybrid_positioner.hpp"
#include "arwain/velocity_prediction.hpp"

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

void publish_positions_on_websocket(arwain::WebSocketServer& server, UUBLA::Network& uubla)
{
    Message<PositionData> message;

    if (arwain::config.pos_to_publish == "uubla")
    {
        auto data = uubla.get_nodes();
        for (auto& [k, v] : data)
        {
            Vector3 pos = v.get_position();
            message.set_header({v.id(), MessageType::position, {123, 123}});
            message.set_data({pos.x, pos.y, pos.z, v.position_fixed() ? 1 : 0});
            // server.add_history(message);
            server.send_message(message.to_string());
        }
    }
    else if (arwain::config.pos_to_publish == "hybrid")
    {
        auto hyb = ServiceManager::get_service<HybridPositioner>(HybridPositioner::service_name);
        if (hyb)
        {
            auto pos = hyb->get_position();
            message.set_header({arwain::config.node_id, MessageType::position, {123, 123}});
            message.set_data({pos.x, pos.y, pos.z, 0});
            server.send_message(message.to_string());
        }
    }
    else if (arwain::config.pos_to_publish == "inertial")
    {
        auto inferrer = ServiceManager::get_service<PositionVelocityInference>(PositionVelocityInference::service_name);
        if (inferrer)
        {
            auto pos = inferrer->get_position();
            message.set_header({arwain::config.node_id, MessageType::position, {123, 123}});
            message.set_data({pos.x, pos.y, pos.z, 0});
            server.send_message(message.to_string());
        }
    }
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

std::unique_ptr<arwain::WebSocketServer> websocket_server;
std::unique_ptr<arwain::WebSocketServer> dash_server;

void dash_messages_callback(arwain::WebSocketServer* ws, std::shared_ptr<WssServer::Connection> cn, std::string& msg)
{
    Header header;
    header.deserialise(msg);

    switch (header.type)
    {
        case MessageType::configuration:
        {
            Message<ConfigurationData> config_message;
            auto& data = config_message.deserialize(msg);

            if (!std::isnan(data.spring_factor))
            {
                UUBLA::Configuration::set_overtight_spring_factor(data.spring_factor);
            }
            if (!std::isnan(data.maximum_expected_rssi))
            {
                UUBLA::Configuration::set_max_expected_rssi(data.maximum_expected_rssi);
            }
            if (!std::isnan(data.minimum_viable_rssi))
            {
                UUBLA::Configuration::set_min_viable_rssi(data.minimum_viable_rssi);
            }
            if (!std::isnan(data.position_gain))
            {
                UUBLA::Configuration::set_position_gain(data.position_gain);
            }
            break;
        }
        default:
            // Nothing else handled on this socket
            break;
    }
}

UublaWrapper::UublaWrapper()
{
    ServiceManager::register_service(this, service_name);

    // The websocket server exists to receive position data from the Unity front end.
    // Upon reception, data is fed to the UUBLA instance running here and there it is
    // used for solving for tag position.
    websocket_server = std::make_unique<arwain::WebSocketServer>(8081, state_messages_callback);
    dash_server = std::make_unique<arwain::WebSocketServer>(8082, dash_messages_callback);
    core_setup();
    job_thread = std::jthread{std::bind_front(&UublaWrapper::run, this)};
}

UublaWrapper::~UublaWrapper()
{
    UUBLA::run_flag = false;
    ServiceManager::unregister_service(service_name);
    // UUBLA::Events::serial_event.remove_callback(eventkey_serial);
    // UUBLA::Events::position_event.remove_callback(eventkey_position);
}

void UublaWrapper::core_setup()
{
    m_uubla = std::make_unique<UUBLA::Network>();
    // TODO Make this optional from config file
    m_uubla->force_plane(arwain::config.force_z_zero);
    l_uubla = m_uubla.get();
}

void UublaWrapper::run_idle()
{
    sleep_ms(10);
}

void UublaWrapper::run()
{
    // if (!arwain::config.use_uwb_positioning || !(arwain::config.node_id < 10))
    // {
    //     return;
    // }

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

void pin_thread(std::jthread& th, int core_number);

void UublaWrapper::setup_inference()
{
    m_uubla->set_ewma_gain(0.1);
    // m_uubla->add_node_callback = inform_new_uubla_node; // Replaced with event registrations.
    // m_uubla->remove_node_callback = inform_remove_uubla_node; // Replaced with event registrations.
    serial_reader_th = std::jthread{serial_reader_fn, m_uubla.get(), arwain::config.uubla_serial_port, 115200};
    pin_thread(serial_reader_th, 3);
}

std::atomic<bool> runflag = true;

bool UublaWrapper::cleanup_inference()
{
    UUBLA::run_flag = false;
    return true;
}

void publish_inertial_on_uwb()
{
    auto inf = ServiceManager::get_service<PositionVelocityInference>(PositionVelocityInference::service_name);
    if (inf)
    {
        auto pos = inf->get_position();
        UUBLA::add_to_send_queue({pos, arwain::config.node_id});
    }
}

TimeStamp get_time()
{
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    return {
        std::chrono::duration_cast<std::chrono::seconds>(now).count(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1'000'000'000
    };
}

std::string state_string(UUBLA::Network* uubla)
{
    JsonObject message;

    auto time = get_time();
    message["header"]["message_type"] = MessageType::diagnostic;
    message["header"]["Node_ID"] = 0xEF;
    message["header"]["time_stamp"]["seconds"] = time.seconds;
    message["header"]["time_stamp"]["nanoseconds"] = time.nanoseconds;

    for (auto& [trusting_node_name, node] : uubla->get_nodes())
    {
        for (auto& [trusted_node_name, trust] : node.beacon_trust)
        {
            message["data"]["trust"][trusting_node_name][trusted_node_name] = trust;
            if (node.beacon_trust.size() > 0)
            {
                message["data"]["rssi"][trusting_node_name][trusted_node_name] = uubla->distances[trusted_node_name+trusting_node_name].rssi;
                message["data"]["rssi_band"][trusting_node_name][trusted_node_name] = UUBLA::bandify_rssi(uubla->distances[trusted_node_name+trusting_node_name].rssi);
                message["data"]["range"][trusting_node_name][trusted_node_name] = uubla->distances[trusted_node_name+trusting_node_name].distance;
            }
        }
    }
    for (auto& [node_name, node] : uubla->get_nodes())
    {
        auto pos = node.get_position();
        message["data"]["position"][node_name]["x"] = pos.x;
        message["data"]["position"][node_name]["y"] = pos.y;
        message["data"]["position"][node_name]["z"] = pos.z;
    }

    return message.dump();
}

void UublaWrapper::run_inference()
{
    setup_inference();

    // TODO: Get the framerate (30?) from somewhere sensible, or hard code?
    IntervalTimer<std::chrono::milliseconds> timer{1000 / (15 /* hz */)};

    auto last_count = timer.count();
    auto frame_counter = 0;
    while (mode == arwain::OperatingMode::Inference)
    {
        frame_counter++;

        if (arwain::config.use_uwb_positioning)
        {
            m_uubla->process_queue();
            m_uubla->solve_map();
            m_uubla->process_callbacks();
            auto now_count = timer.count();
            // TODO The maths below is because interval timer.count doesn't return the milisecond count as it should.
            // Consider also the timing in other inetval timer locations before making changes.
            arwain::Events::new_uwb_position_event.invoke({get_own_position(), (now_count - last_count) / 1000.0 / 1000.0 / 1000.0});
            last_count = now_count;
        }

        if (frame_counter % 3 == 0)
        {
            dash_server->send_message(state_string(m_uubla.get()));
        }

        // We still want to publish positions over WebSocket, even if we aren't using UWB positioning.
        publish_positions_on_websocket(*websocket_server, *m_uubla);
        // publish_inertial_on_uwb();
        timer.await();
    }

    cleanup_inference();
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
    return m_uubla->get_node_position(
        UUBLA::Node::name_from_int(
            arwain::config.node_id
        )
    );
}

Vector3 UublaWrapper::get_node_position(const std::string& node_name) const
{
    return m_uubla->get_node_position(node_name);
}
