#include <cmath>

#include "floor_tracker.hpp"
#include "vector3.hpp"

/** \brief Constructor. Assumes initial position = 0. */
arwain::FloorTracker::FloorTracker(int window_size_, double drift_threshold_, double min_separation_)
: window_size(window_size_), min_separation(min_separation_), drift_threshold(drift_threshold_)
{
    track.push_back({0, 0, 0});
}

/** \brief Alternate constructor, allowing specification of initial position. */
arwain::FloorTracker::FloorTracker(int window_size_, double drift_threshold_, double min_separation_, const Vector3& initial_position_)
: window_size(window_size_), min_separation(min_separation_), drift_threshold(drift_threshold_)
{
    track.push_back(initial_position_);
}

/** \brief Computes the climbing gradient between two 3-vectors.
 * \param v1 Vector 1.
 * \param v2 Vector 2.
 * \return Gradient.
 */
static double vertical_gradient(const Vector3& v1, const Vector3& v2)
{
    Vector3 delta_vec = v2 - v1;
    double vertical_change = v2.z - v1.z;
    delta_vec.z = 0;
    double horizontal_change = delta_vec.magnitude();
    return vertical_change / horizontal_change;
}

bool arwain::FloorTracker::update(const Vector3& position)
{
    // If the new position is very close to the old one, do not add it to the track.
    // We require meaningful spatial separation to detect corners using this method.
    if ((track.back() - position).magnitude() < min_separation)
    {
        return false;
    }

    track.push_back(position);

    if (track.size() < window_size)
    {
        // std::cout << "Skipped; insufficient information" << std::endl;
        return false;
    }

    auto& anchor = track[0];
    auto& start = track[1];
    auto& end = track.back();

    // If gradient between start and end less than threshold, offset all except anchor by (point - start + anchor).
    if (std::abs(vertical_gradient(start, end)) < drift_threshold)
    {
        for (unsigned int i = 1; i < track.size(); i++)
        {
            track[i].z = track[i].z - start.z + anchor.z;
        }
    }

    tracked_position = track[0];

    track.pop_front();
    return true;
}
