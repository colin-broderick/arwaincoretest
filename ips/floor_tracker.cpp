#include "floor_tracker.hpp"
#include "vector3.hpp"

arwain::FloorTracker::FloorTracker(int window_size_, double min_separation_, double drift_threshold_)
: window_size(window_size_), min_separation(min_separation_), drift_threshold(drift_threshold_)
{
    track.push_back({0, 0, 0});
}

/** \brief Computes the climbing gradient between two 3-vectors.
 * \param v1 Vector 1.
 * \param v2 Vector 2.
 * \return Gradient.
 */
static double vertical_gradient(const vector3& v1, const vector3& v2)
{
    vector3 delta_vec = v2 - v1;
    double vertical_change = v2.z - v1.z;
    delta_vec.z = 0;
    double horizontal_change = delta_vec.magnitude();
    return vertical_change / horizontal_change;
}

void arwain::FloorTracker::update(const vector3& position)
{
    // If the new position is very close to the old one, do not add it to the track.
    // We require meaningful spatial separation to detect corners using this method.
    if ((track.back() - position).magnitude() < min_separation)
    {
        return;
    }

    track.push_back(position);

    if (track.size() < window_size)
    {
        std::cout << "Skipped; insufficient information" << std::endl;
        return;
    }

    auto& anchor = track[0];
    auto& start = track[1];
    auto& end = track.back();

    // TODO if gradient between start and end less than threshold, offset all except anchor by (point - start + anchor).
    if (vertical_gradient(start, end) < drift_threshold)
    {
        for (unsigned int i = 1; i < track.size(); i++)
        {
            track[i].z = track[i].z - start.z + anchor.z;
        }
    }

    tracked_position = track[0];
}
