#ifndef PREDICT_VELOCITY_H
#define PREDICT_VELOCITY_H

#include "arwain.hpp"

// void predict_velocity();

namespace PositionVelocityInference
{
    std::tuple<bool, std::string> set_mode(arwain::OperatingMode new_mode);
    bool shutdown();
    void join();
    bool init();
}

#endif
