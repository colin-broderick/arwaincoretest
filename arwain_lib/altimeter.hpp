#ifndef _ARWAIN_ALTIMETER_HPP
#define _ARWAIN_ALTIMETER_HPP

#include <tuple>

#include "arwain.hpp"

namespace Altimeter
{
    bool init();
    bool shutdown();
    void join();
    std::tuple<bool, std::string> set_mode(arwain::OperatingMode new_mode);
    arwain::OperatingMode get_mode();
}

#endif
