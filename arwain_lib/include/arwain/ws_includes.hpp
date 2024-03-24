// #ifndef _MESSAGE_HPP
// #define _MESSAGE_HPP

// #include <nlohmann/json.hpp>

// class JsonObject : public nlohmann::json
// {
//     // Empty wrapper in case we move to a new JSON library.
// };

// struct TimeStamp
// {
//     int64_t seconds = 0;
//     int64_t nanoseconds = 0;
// };

// enum class MessageType
// {
//     Position,
//     Rotation,
//     Stance,
//     HistoryRequest,
//     Configuration,
//     Diagnostic,
//     Command
// };

// enum class Stance
// {
//     Walking,
//     Running,
//     Standing,
//     Crawling,
//     Freefall
// };

/** \brief Defines valid command that can be sent over the WebSocket connection.
 * Must be identically defined for all clients and servers that wish to use this
 * interface.
 */
// enum class WSCommand
// {
//     StartInertialTracking,
//     StopInertialTracking
// };

// class Header
// {
//     public:
//         void set_header(int id, MessageType m_type, TimeStamp time);
//         void deserialise(std::string message);

//     public:
//         int node_id = 0;
//         MessageType type;
//         TimeStamp stamp;
// };

// struct StanceData
// {
//     Stance stance; 
// };

// struct PositionData
// {
//     double X = 0.0;
//     double Y = 0.0;
//     double Z = 0.0;
//     int is_fixed = 0;
// };

// struct CommandData
// {
    // WSCommand command;
// };

// struct ConfigurationData
// {
//     double spring_factor = std::nan("");
//     double position_gain = std::nan("");
//     double minimum_viable_rssi = std::nan("");
//     double maximum_expected_rssi = std::nan("");
// };

// template <class DataClass>
// class Message
// {
//     public:
//         JsonObject serialise();
//         DataClass& deserialize(const std::string& message);
//         void set_data(DataClass data);
//         DataClass& get_data()
//         {
//             return m_data;
//         }
//         void set_header(Header header)
//         {
//             m_header = header;
//             m_json_message["header"]["message_type"] = header.type;
//             m_json_message["header"]["Node_ID"] = header.node_id;
//             m_json_message["header"]["time_stamp"]["seconds"] = header.stamp.seconds;
//             m_json_message["header"]["time_stamp"]["nanoseconds"] = header.stamp.nanoseconds;
//         }
//         Header& get_header()
//         {
//             return m_header;
//         }
//         std::string to_string()
//         {
//             return m_json_message.dump();
//         }

//     private:
//         Header m_header;
//         DataClass m_data;
//         JsonObject m_json_message;
// };

// #endif