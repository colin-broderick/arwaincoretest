#ifndef _ARWAIN_CALIBRATION_HPP
#define _ARWAIN_CALIBRATION_HPP

#include <NumCpp.hpp>

#include "vector3.hpp"

namespace arwain
{
    class MagnetometerCalibrator
    {
        public:
            void feed(const Vector3& reading);
            std::tuple<std::vector<double>, std::vector<std::vector<double>>> solve();

        private:
            nc::NdArray<double> xyz;

    };
}

#endif
