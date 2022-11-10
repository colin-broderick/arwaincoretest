#ifndef _ARWAIN_CORNER_DETECTOR_HPP
#define _ARWAIN_CORNER_DETECTOR_HPP

#include <deque>

#include "vector3.hpp"

namespace arwain
{
    /** \brief Attempts to identify and mark sharp corners in a 2D path. Does not use orientation
     * directly. Uses a subsample of the plotted path. */
    class CornerDetector
    {
        public:
            CornerDetector(int window_size_, double detection_angle_, double min_separation_);
            bool update(Vector3 position);
            
            Vector3 detection_location;

        private:
            int window_size;
            double detection_angle;
            double min_separation;
            std::deque<Vector3> track;

            double radians_to_degrees(double radians);
    };
}

#endif
