#ifndef _ARWAIN_CORNER_DETECTOR_HPP
#define _ARWAIN_CORNER_DETECTOR_HPP

#include <deque>

#include "vector3.hpp"

namespace arwain
{
    class CornerDetector
    {
        public:
            CornerDetector(int window_size_, double detection_angle_, double min_separation_);
            bool update(vector3 position);
            
            vector3 detection_location;

        private:
            int window_size;
            double detection_angle;
            double min_separation;
            std::deque<vector3> track;
    };
}

#endif
