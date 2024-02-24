#include <sstream>
#include <fstream>

#include "arwain/ws_includes.hpp"

/***************************************************************************************/
/************** HEADERS ****************************************************************/
/***************************************************************************************/

void Header::set_header(int id, MessageType m_type, TimeStamp time)
{
    node_id = id;
    type = m_type;
    stamp = time;
}

void Header::deserialise(std::string message)
{
    JsonObject json;
    std::stringstream ss;
    ss << message;
    ss >> json;

    node_id = json["header"]["Node_ID"];
    stamp.seconds = json["header"]["time_stamp"]["seconds"];
    stamp.nanoseconds = json["header"]["time_stamp"]["nanoseconds"];
    type = json["header"]["message_type"];
}

/***************************************************************************************/
/************** POSITION DATA SPECIALIZATIONS ******************************************/
/***************************************************************************************/

template <>
void Message<PositionData>::set_data(PositionData pos)
{
    m_data = pos;
    m_json_message["data"]["X"] = pos.X;
    m_json_message["data"]["Y"] = pos.Y;
    m_json_message["data"]["Z"] = pos.Z;
    m_json_message["data"]["is_fixed"] = pos.is_fixed;
}

template <>
JsonObject Message<PositionData>::serialise()
{
    JsonObject json;

    json["header"]["message_type"] = m_header.type;
    json["header"]["Node_ID"] = m_header.node_id;
    json["header"]["time_stamp"]["seconds"] = m_header.stamp.seconds;
    json["header"]["time_stamp"]["nanoseconds"] = m_header.stamp.nanoseconds;

    json["data"]["X"] = m_data.X;
    json["data"]["Y"] = m_data.Y;
    json["data"]["Z"] = m_data.Z;
    json["data"]["is_fixed"] = m_data.is_fixed;

    return json;
}

template<>
PositionData& Message<PositionData>::deserialize(const std::string& message)
{
    JsonObject json;
    std::stringstream ss;
    ss << message;
    ss >> json;

    m_header.node_id = json["header"]["Node_ID"];
    m_header.stamp.seconds = json["header"]["time_stamp"]["seconds"];
    m_header.stamp.nanoseconds = json["header"]["time_stamp"]["nanoseconds"];
    m_header.type = json["header"]["message_type"];

    m_data.X = json["data"]["X"];
    m_data.Y = json["data"]["Y"];
    m_data.Z = json["data"]["Z"];
    m_data.is_fixed = json["data"]["is_fixed"];

    return m_data;
}

/***************************************************************************************/
/************** STANCE DATA SPECIALIZATIONS ********************************************/
/***************************************************************************************/

template <>
JsonObject Message<StanceData>::serialise()
{
    JsonObject json;

    json["header"]["message_type"] = m_header.type;
    json["header"]["Node_ID"] = m_header.node_id;
    json["header"]["time_stamp"]["seconds"] = m_header.stamp.seconds;
    json["header"]["time_stamp"]["nanoseconds"] = m_header.stamp.nanoseconds;

    json["data"]["stance"] = m_data.stance;

    return json;
}

template<>
StanceData& Message<StanceData>::deserialize(const std::string& message)
{
    JsonObject json;
    std::stringstream ss;
    ss << message;
    ss >> json;

    m_header.node_id = json["header"]["Node_ID"];
    m_header.stamp.seconds = json["header"]["time_stamp"]["seconds"];
    m_header.stamp.nanoseconds = json["header"]["time_stamp"]["nanoseconds"];
    m_header.type = json["header"]["message_type"];

    m_data.stance = json["data"]["stance"];

    return m_data;
}

template <>
void Message<StanceData>::set_data(StanceData stance)
{
    m_json_message["data"]["stance"] = stance.stance;
}

/***************************************************************************************/
/************** CONFIG DATA SPECIALIZATIONS ********************************************/
/***************************************************************************************/

template<>
JsonObject Message<ConfigurationData>::serialise()
{
    throw std::runtime_error{"Not implemented: JsonObject Message<ConfigurationData>::serialise()"};
}

template<> 
ConfigurationData& Message<ConfigurationData>::deserialize(const std::string& message)
{
    // todo
    JsonObject json;
    std::stringstream ss;
    ss << message;
    ss >> json;

    m_header.node_id = json["header"]["Node_ID"];
    m_header.stamp.seconds = json["header"]["time_stamp"]["seconds"];
    m_header.stamp.nanoseconds = json["header"]["time_stamp"]["nanoseconds"];
    m_header.type = json["header"]["message_type"];

    // We check nullptrs because a given packet may contain only partial configuration instructions.
    if (json["data"]["spring_factor"] != nullptr)
    {
        m_data.spring_factor = json["data"]["spring_factor"];
    }
    if (json["data"]["maximum_expected_rssi"] != nullptr)
    {
        m_data.maximum_expected_rssi = json["data"]["maximum_expected_rssi"];
    }
    if (json["data"]["minimum_viable_rssi"] != nullptr)
    {
        m_data.minimum_viable_rssi = json["data"]["minimum_viable_rssi"];
    }
    if (json["data"]["position_gain"] != nullptr)
    {
        m_data.position_gain = json["data"]["position_gain"];
    }

    return m_data;
}

template<>
void Message<ConfigurationData>::set_data(ConfigurationData data)
{
    throw std::runtime_error{"Not implemented: void Message<ConfigurationData>::set_data(ConfigurationData data)"};
}



#include <arwain/websocket.hpp>
#include <uubla/node.hpp>
#include <uubla/network.hpp>

#include "arwain/service_manager.hpp"
#include "arwain/hybrid_positioner.hpp"
#include "arwain/velocity_prediction.hpp"
#include "arwain/uwb_reader.hpp"
#include "arwain/ws_messenger.hpp"

std::unique_ptr<arwain::WebSocketServer> websocket_server;
std::unique_ptr<arwain::WebSocketServer> dash_server;

TimeStamp get_time()
{
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    return {
        std::chrono::duration_cast<std::chrono::seconds>(now).count(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count() % 1'000'000'000
    };
}

void send_network_state_message(const std::string& msg)
{
    if (dash_server)
    {
        dash_server->send_message(msg);
    }
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

void position_callback(int nodeID, double x, double y, double z)
{
    auto u = ServiceManager::get_service<UublaWrapper>(UublaWrapper::service_name);
    if (u == nullptr)
    {
        return;
    }

    if (!u->network_contains(nodeID))
    {
        return;
    }

    u->fix_node_at(nodeID, {x, y, z});
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

void publish_positions_on_websocket(UUBLA::Network& uubla)
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
            websocket_server->send_message(message.to_string());
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
            websocket_server->send_message(message.to_string());
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
            websocket_server->send_message(message.to_string());
        }
    }
}

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

bool once_only = true;
WsMessenger::WsMessenger()
{
    if (once_only)
    {
        // TODO Currently this initializes file-local data. Wrap everything up into a class.
        once_only = false;
        websocket_server = std::make_unique<arwain::WebSocketServer>(8081, state_messages_callback);
        dash_server = std::make_unique<arwain::WebSocketServer>(8082, dash_messages_callback);
    }
    else
    {
        throw std::runtime_error{"This is supposed to only exist once."};
    }
}
