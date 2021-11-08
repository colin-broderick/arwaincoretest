#ifndef _ARWAIN_IPS_FLOOR_TRACKER_HPP
#define _ARWAIN_IPS_FLOOR_TRACKER_HPP

#include <string>
#include <deque>

#include <vector3.hpp>

namespace arwain
{
    class FloorTracker
    {
        public:
            FloorTracker(int window_size_, double min_separation_, double drift_threshold_);
            void update(const vector3& position);
            vector3 tracked_position;

        private:
            std::deque<vector3> track;
            int window_size;
            double min_separation;
            double drift_threshold;
    };
}

#endif
