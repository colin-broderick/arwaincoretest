#ifndef _ARWAIN_STD_OUTPUT_HPP
#define _ARWAIN_STD_OUTPUT_HPP

#include <tuple>

namespace DebugPrints
{
    bool init();
    bool shutdown();
    void join();
    std::tuple<bool, std::string> set_mode(arwain::OperatingMode new_mode);
}

#endif
