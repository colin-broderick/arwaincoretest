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
            int get_sphere_coverage_quality();

        private:
            int feed_count = 0;
            int sphere_coverage_quality = 0;
            int region_sample_count[100] = {0};
            nc::NdArray<double> xyz;
            Vector3 region_sample_value[100] = {{0, 0, 0}};

    };
}

#endif
