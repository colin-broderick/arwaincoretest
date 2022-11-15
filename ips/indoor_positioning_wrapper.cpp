#include <string>
#include <mutex>
#include <deque>
#include <thread>

#include "indoor_positioning_wrapper.hpp"
#include "floor_tracker.hpp"
#include "corner_detector.hpp"
#include "logger.hpp"
#include "vector3.hpp"
#include "exceptions.hpp"
#include "arwain_thread.hpp"
#include "arwain.hpp"

namespace IndoorPositioningSystem
{
    namespace // private
    {
        ArwainThread job_thread;

        arwain::Logger corner_log;
        arwain::Logger tracked_floor_log;

        arwain::CornerDetector corner_detector{11, 115.0, 0.20}; // 11 * 0.2 means a window of 11 points separated by at least 20 cm each, so about 2 m total.
        arwain::FloorTracker floor_tracker{5, 0.10, 0.20}; // UK stairs are gradient approx. 0.9.

        void core_setup()
        {
        }

        void setup_inference()
        {
            corner_log.open(arwain::folder_date_string + "/corner_log.txt");
            corner_log << "time x y z\n";
            tracked_floor_log.open(arwain::folder_date_string + "/tracked_floors.txt");
            tracked_floor_log << "time x, y, z\n" ;
        }

        void cleanup_inference()
        {
            corner_log.close();
            tracked_floor_log.close();
        }

        void run_inference()
        {
            setup_inference();

            auto time = std::chrono::high_resolution_clock::now();
            std::chrono::milliseconds interval{arwain::Intervals::IPS_INTERVAL};

            while (arwain::system_mode == arwain::OperatingMode::Inference)
            {
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

                // Wait until next tick.
                time = time + interval;
                std::this_thread::sleep_until(time);
            }

            cleanup_inference();
        }

        void run_idle()
        {
            sleep_ms(10);
        }

        void run()
        {
            while (arwain::system_mode != arwain::OperatingMode::Terminate)
            {
                switch (arwain::system_mode)
                {
                case arwain::OperatingMode::Inference:
                    run_inference();
                    break;
                
                default:
                    run_idle();
                    break;
                }
            }
        }
    }

    // Public

    bool init()
    {
        if (!arwain::config.use_ips)
        {
            return false;
        }
        core_setup();
        job_thread = ArwainThread{IndoorPositioningSystem::run, "arwain_ips_th"};
        return true;
    }

    void join()
    {
        job_thread.join();
    }
}

void IndoorPositioningSystem::IndoorPositioningWrapper::update(const double &time, const double &x, const double &y, const double &z)
{
    // TODO Pass the parameters to the inner IPS object and put results in
    // m_x, m_y, m_z.
}

Vector3 IndoorPositioningSystem::IndoorPositioningWrapper::getPosition()
{
    return {m_x, m_y, m_z};
}

double IndoorPositioningSystem::IndoorPositioningWrapper::getX()
{
    return m_x;
}

double IndoorPositioningSystem::IndoorPositioningWrapper::getY()
{
    return m_y;
}

double IndoorPositioningSystem::IndoorPositioningWrapper::getZ()
{
    return m_z;
}
