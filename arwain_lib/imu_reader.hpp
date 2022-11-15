#ifndef _ARWAIN_IMU_READER_HPP
#define _ARWAIN_IMU_READER_HPP

#include <functional>

#include "arwain.hpp"

namespace ImuProcessing
{
    void set_post_gyro_calibration_callback(std::function<void()> func);
    void join();
    bool init();
}

// void imu_reader();

#endif
