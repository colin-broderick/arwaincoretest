#include <string>
#include <mutex>
#include <deque>
#include <thread>

#include "indoor_positioning_wrapper.hpp"
#include "floor_tracker.hpp"
#include "corner_detector.hpp"
#include "logger.hpp"
#include "vector3.hpp"
#include "arwain.hpp"

/** \brief Indoor positioning system for recognising and snapping to stairs, floors, etc.
 * Runs as thread.
 */
void indoor_positioning()
{
    if (!arwain::config.use_ips)
    {
         return;
    }

    arwain::CornerDetector corner_detector{11, 115.0, 0.20}; // 11 * 0.2 means a window of 11 points separated by at least 20 cm each, so about 2 m total.
    arwain::FloorTracker floor_tracker{5, 0.10, 0.20}; // UK stairs are gradient approx. 0.9.

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::Inference:
            {
                arwain::Logger corner_log;
                arwain::Logger tracked_floor_log;
                corner_log.open(arwain::folder_date_string + "/corner_log.txt");
                corner_log << "time x y z\n";
                tracked_floor_log.open(arwain::folder_date_string + "/tracked_floors.txt");
                tracked_floor_log << "time x, y, z\n" ;

                auto time = std::chrono::high_resolution_clock::now();
                std::chrono::milliseconds interval{arwain::Intervals::IPS_INTERVAL};

                while (arwain::system_mode == arwain::OperatingMode::Inference)
                {

                    // Wait until next tick.
                    time = time + interval;
                    std::this_thread::sleep_until(time);

                    Vector3 new_position;
                    new_position = arwain::Buffers::POSITION_BUFFER.back();

                    if (corner_detector.update(new_position))
                    {
                        corner_log << time.time_since_epoch().count() << " "
                                << corner_detector.detection_location.x << " "
                                << corner_detector.detection_location.y << " "
                                << corner_detector.detection_location.z << "\n";
                    }

                    floor_tracker.update(new_position);
                    // std::cout << time.time_since_epoch().count() << " "
                    //           << floor_tracker.tracked_position.x << " "
                    //           << floor_tracker.tracked_position.y << " "
                    //           << floor_tracker.tracked_position.z << "\n";
                }
                break;
            }
            default:
            {
                sleep_ms(10);
                break;
            }
        }
    }
}

void arwain::IndoorPositioningWrapper::update(const double &time, const double &x, const double &y, const double &z)
{
    // TODO Pass the parameters to the inner IPS object and put results in
    // m_x, m_y, m_z.
}

Vector3 arwain::IndoorPositioningWrapper::getPosition()
{
    return {m_x, m_y, m_z};
}

double arwain::IndoorPositioningWrapper::getX()
{
    return m_x;
}

double arwain::IndoorPositioningWrapper::getY()
{
    return m_y;
}

double arwain::IndoorPositioningWrapper::getZ()
{
    return m_z;
}
