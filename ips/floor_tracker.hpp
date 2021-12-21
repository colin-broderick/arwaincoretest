#ifndef _ARWAIN_IPS_FLOOR_TRACKER_HPP
#define _ARWAIN_IPS_FLOOR_TRACKER_HPP

#include <string>
#include <deque>

#include "vector3.hpp"

namespace arwain
{
    /** \brief Attempts to smooth out drift and minor elevation changes in a 3D path, while
     * preserving true elevation changes such as those from stairs, ladders, etc.
     * 
     * We apply a sliding window to a subsample of the 3D path. If the spatial gradient
     * (importantly NOT the temporal gradient) between the start and end of the segment
     * is below a given threshold, we adjust all points in the path segment to be at the
     * same height as the first. This way true elevation changes are retained, while those
     * from drift and minor vertical motions are discarded.
     */
    class FloorTracker
    {
        public:
            FloorTracker(int window_size_, double drift_threshold_, double min_separation_);
            FloorTracker(int window_size_, double drift_threshold_, double min_separation_, const Vector3& initial_position_);
            void update(const Vector3& position);
            Vector3 tracked_position = {0, 0, 0};

        private:
            std::deque<Vector3> track;
            int window_size;
            double min_separation;
            double drift_threshold;
    };
}

#endif
