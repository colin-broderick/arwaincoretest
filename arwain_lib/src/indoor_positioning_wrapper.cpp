#include <string>
#include <mutex>
#include <deque>
#include <thread>

#include "indoor_positioning_wrapper.hpp"
#include "floor_tracker.hpp"
#include "corner_detector.hpp"
#include "timers.hpp"
#include "logger.hpp"
#include "vector3.hpp"
#include "exceptions.hpp"
#include "arwain_thread.hpp"
#include "arwain.hpp"

IndoorPositioningSystem::IndoorPositioningSystem()
{
    init();
}

void IndoorPositioningSystem::core_setup()
{
}

void IndoorPositioningSystem::setup_inference()
{
    corner_log.open(arwain::folder_date_string + "/corner_log.txt");
    corner_log << "time x y z\n";
    tracked_floor_log.open(arwain::folder_date_string + "/tracked_floors.txt");
    tracked_floor_log << "time x, y, z\n" ;
}

void IndoorPositioningSystem::cleanup_inference()
{
    corner_log.close();
    tracked_floor_log.close();
}

void IndoorPositioningSystem::run_inference()
{
    setup_inference();

    Timers::IntervalTimer<std::chrono::milliseconds> loop_scheduler{arwain::Intervals::IPS_INTERVAL, "arwain_ips_run_infer"};

    while (arwain::system_mode == arwain::OperatingMode::Inference)
    {
        Vector3 new_position;
        new_position = arwain::Buffers::POSITION_BUFFER.back();

        if (corner_detector.update(new_position))
        {
            corner_log << loop_scheduler.count() << " "
                        << corner_detector.detection_location.x << " "
                        << corner_detector.detection_location.y << " "
                        << corner_detector.detection_location.z << "\n";
        }

        floor_tracker.update(new_position);

        // Wait until next tick.
        loop_scheduler.await();
    }

    cleanup_inference();
}

void IndoorPositioningSystem::run_idle()
{
    sleep_ms(10);
}

void IndoorPositioningSystem::run()
{
    if (!arwain::config.use_ips)
    {
        return;
    }
    
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

bool IndoorPositioningSystem::init()
{
    core_setup();
    job_thread = ArwainThread{&IndoorPositioningSystem::run, "arwain_ips_th", this};
    return true;
}

void IndoorPositioningSystem::join()
{
    while (!job_thread.joinable())
    {
        sleep_ms(1);
    }
    if (job_thread.joinable())
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
