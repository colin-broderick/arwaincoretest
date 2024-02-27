#ifndef _ARWAIN_WS_MESSENGER_HPP
#define _ARWAIN_WS_MESSENGER_HPP

#include "arwain/service_interface.hpp"

class WsMessenger : public IArwainService
{
    public:
        WsMessenger();
        ~WsMessenger();

    public:
        static const inline std::string service_name = "HybridPositioner";
};

#endif
