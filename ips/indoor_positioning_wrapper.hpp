#ifndef INDOOR_POSITIONING_WRAPPER_H
#define INDOOR_POSITIONING_WRAPPER_H

#include "vector3.hpp"
#include "arwain.hpp"

namespace IndoorPositioningSystem
{
    std::tuple<bool, std::string> set_mode(arwain::OperatingMode new_mode);
    bool shutdown();
    void join();
    bool init();

    class IndoorPositioningWrapper
    {
        private:
            double m_x = 0;
            double m_y = 0;
            double m_z = 0;

            // TODO Create inner IPS object with Ross's rust library.

        public:
            void update(const double &time, const double &x, const double &y, const double &z);
            Vector3 getPosition();
            double getX();
            double getY();
            double getZ();
    };
}

#endif
