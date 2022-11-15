#ifndef _ARWAIN_IMU_READER_HPP
#define _ARWAIN_IMU_READER_HPP

#include <functional>

#include "arwain.hpp"

namespace ImuProcessing
{
    std::tuple<bool, std::string> set_mode(arwain::OperatingMode new_mode, std::function<void()> func);
    std::tuple<bool, std::string> set_mode(arwain::OperatingMode new_mode);
    arwain::OperatingMode get_mode();
    bool shutdown();
    void join();
    bool init();
}

// void imu_reader();

#endif
