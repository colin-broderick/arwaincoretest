#include "arwain/corner_detector.hpp"

/** \brief Constructor.
 * \param window_size_ The size of the position sample window over which to check for a corner.
 * \param detection_angle_ The minimum angle which will register as the detection of a corner.
 * \param min_separation_ If successive position measurements differ by less than this value, the new one will be ignored.
 */
arwain::CornerDetector::CornerDetector(int window_size_, double detection_angle_, double min_separation_)
: window_size(window_size_), detection_angle(detection_angle_), min_separation(min_separation_)
{
    track.push_back({0, 0, 0});
}

/** \brief Converts an angle in radians to an angle in degrees.
 *
 * Converts radians to degrees by multiplying by
 * 180.0° (i.e. half a circle in degrees) and then dividing by
 * pi (i.e. half a circle in radians).
 *
 * \param radians The size of the angle to convert in radians
 */
double arwain::CornerDetector::radians_to_degrees(double radians)
{
    //multuplied by 180 degrees and then divided by pi
    return radians * 180.0 / 3.141592653;
}

/** \brief Checks the track to see if a corner can be detected.
 * 
 * The detector chooses an anchor point, then looks forward in time to an end vector,
 * and backward in time to a start vector. If the start and end vectors are separated
 * (relative to the anchor) by an angle of more than detection_angle, a corner is
 * detected. The detector will then step the end point backward, and the start point
 * forward, in order to localise the point as much as possible. When it has been localized
 * the detector returns true, and the calling code can then access detection_location
 * if they need to know the location of the corner.
 * 
 * Window size and call frequency are related. If, for example, you want to look at five
 * second slices of the track, then (window_size, call_frequency) could be paired as, e.g.
 *      (11, 500 ms)
 * This means the update function should be called every 500 ms and will keep the 11 most
 * recent position updates (assuming appropriate separation). This corresponds roughly to
 * five seconds of travel time. We use a window size of 11 rather than to guarantee that
 * there will always be a centre index. Odd number window sizes are to be preferred for
 * this reason, though the detector will function regardless.
 *      Something like (3, 5 ms) would be an unsuitable usage pattern as no meaningful
 * human motion takes place over the course of 15 ms.
 * 
 * \param position The most recent known position.
 * \return Boolean flag indiciated whether a corner was detected or not.
 */
bool arwain::CornerDetector::update(Vector3 position)
{
    // We are only looking for corners in the xy plane.
    position.z = 0;

    // If the new position is very close to the old one, do not add it to the track.
    // We require meaningful spatial separation to detect corners using this method.
    if ((track.back() - position).magnitude() < min_separation)
    {
        // std::cout << "Skipped; insufficient motion" << std::endl;
        return false;
    }

    track.push_back(position);

    if (track.size() < window_size)
    {
        // std::cout << "Skipped; insufficient information" << std::endl;
        return false;
    }

    int start_index = 0;
    int end_index = window_size - 1;
    int halfway_index = (end_index - start_index) / 2;

    // Check if a turn of detection_angle degrees or greater is evident in the track.
    Vector3 anchor = track[halfway_index];
    Vector3 start = track[start_index] - anchor;
    Vector3 end = track[end_index] - anchor;
    double angle = radians_to_degrees(Vector3::angle_between(start, end));

    std::cout << angle << std::endl;

    if (angle < detection_angle)
    {
        while (start_index + 2 < end_index) // start and end indices must always be separated by at least 2 to guarantee there is an anchor index in between.
        {
            end_index--;
            halfway_index = start_index + (end_index - start_index) / 2;
            anchor = track[halfway_index];
            start = track[start_index] - anchor;
            end = track[end_index] - anchor;
            double new_angle = radians_to_degrees(Vector3::angle_between(start, end));
            if (new_angle > detection_angle) 
            {
                angle = new_angle;
            }
            else
            {
                break;
            }
        }
        while (start_index + 2 < end_index)
        {
            start_index--;
            halfway_index = start_index + (end_index - start_index) / 2;
            anchor = track[halfway_index];
            start = track[start_index] - anchor;
            end = track[end_index] - anchor;
            double new_angle = radians_to_degrees(Vector3::angle_between(start, end));
            if (new_angle > detection_angle) 
            {
                angle = new_angle;
            }
            else
            {
                break;
            }
        }

        detection_location = anchor;
    }

    track.pop_front();
    return true;
}
