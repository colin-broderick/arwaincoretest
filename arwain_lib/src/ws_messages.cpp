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