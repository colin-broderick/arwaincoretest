#ifndef _ARWAIN_UWB_READER_HPP
#define _ARWAIN_UWB_READER_HPP

namespace UublaWrapper
{
    bool init();
    void join();
    bool shutdown();
    std::tuple<bool, std::string> set_mode(arwain::OperatingMode new_mode);
    double get_distance(const int position);
}

#endif
