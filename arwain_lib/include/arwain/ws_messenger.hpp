// #ifndef _ARWAIN_WS_MESSENGER_HPP
// #define _ARWAIN_WS_MESSENGER_HPP

// #include <arwain/websocket.hpp>

// #include "arwain/service_interface.hpp"

// class WsMessenger : public IArwainService
// {
//     public:
//         WsMessenger();
//         ~WsMessenger();

//     public:
//         static const inline std::string service_name = "HybridPositioner";
//         void send_dash_message(const std::string& msg);
//         void publish_positions_on_websocket(UUBLA::Network& uubla);

//     private:
//         std::unique_ptr<arwain::WebSocketServer> websocket_server;
//         std::unique_ptr<arwain::WebSocketServer> dash_server;
//         void publish_uubla(UUBLA::Network& uubla);
//         void publish_hybrid();
//         void publish_inertial();
// };

// #endif
