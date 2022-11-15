#ifndef _ARWAIN_UWB_READER_HPP
#define _ARWAIN_UWB_READER_HPP

namespace UublaWrapper
{
    bool init();
    void join();
    double get_distance(const int position);
}

#endif
