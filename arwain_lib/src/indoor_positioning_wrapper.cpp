#include <string>
#include <mutex>
#include <deque>
#include <thread>

#include "arwain/indoor_positioning_wrapper.hpp"
#include "arwain/floor_tracker.hpp"
#include "arwain/corner_detector.hpp"
#include "arwain/exceptions.hpp"
#include "arwain/thread.hpp"
#include "arwain/arwain.hpp"
#include "arwain/logger.hpp"

#include "timers.hpp"
#include "vector3.hpp"

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

    while (mode == arwain::OperatingMode::Inference)
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
    
    while (mode != arwain::OperatingMode::Terminate)
    {
        switch (mode)
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
    if (job_thread.joinable())
    {
        job_thread.join();
    }
    job_thread = ArwainThread{&IndoorPositioningSystem::run, "arwain_ips_th", this};
    return true;
}

bool IndoorPositioningSystem::join()
{
    while (!job_thread.joinable())
    {
        sleep_ms(1);
    }
    if (job_thread.joinable())
    {
        job_thread.join();
        return true;
    }
    return false;
}

void IndoorPositioningSystem::IndoorPositioningWrapper::update(const double &time, const double &x, const double &y, const double &z)
{
    // TODO Pass the parameters to the inner IPS object and put results in
    // m_x, m_y, m_z.
}

Vector3 IndoorPositioningSystem::IndoorPositioningWrapper::get_position() const
{
    return {m_x, m_y, m_z};
}

double IndoorPositioningSystem::IndoorPositioningWrapper::get_x() const
{
    return m_x;
}

double IndoorPositioningSystem::IndoorPositioningWrapper::get_y() const
{
    return m_y;
}

double IndoorPositioningSystem::IndoorPositioningWrapper::get_z() const
{
    return m_z;
}
