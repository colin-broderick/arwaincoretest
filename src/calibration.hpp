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
            int region_sample_count[100] = {0};
            Vector3 region_sample_value[100] = {{0, 0, 0}};

    };
}

#endif
