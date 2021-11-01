#include <string>
#include <mutex>
#include <deque>
#include <thread>

#include "indoor_positioning_wrapper.hpp"
#include "logger.hpp"
#include "vector3.hpp"
#include "arwain.hpp"

/** \brief Indoor positioning system for recognising and snapping to stairs, floors, etc.
 * Runs as thread.
 */
void indoor_positioning()
{
    // Quit immediately if IPS disabled by configuration file.
    if (!arwain::config.use_indoor_positioning_system)
    {
        return;
    }

    // TODO Create IPS object
    arwain::IndoorPositioningWrapper ips;
    vector3 velocity;
    vector3 position;

    arwain::Logger ips_position_file;

    if (arwain::config.log_to_file)
    {
        ips_position_file.open(arwain::folder_date_string + "/ips_position.txt");
        ips_position_file << "# time x y z" << "\n";
    }

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{arwain::Intervals::INDOOR_POSITIONING_INTERVAL};

    while (!arwain::shutdown)
    {
        { // Get most recent velocity data.
            std::lock_guard<std::mutex> lock{arwain::Locks::VELOCITY_BUFFER_LOCK};
            velocity = arwain::Buffers::VELOCITY_BUFFER.back();
        }

        // Run update and get new position.
        ips.update(
            time.time_since_epoch().count(),
            velocity.x,
            velocity.y,
            velocity.z
        );
        position = ips.getPosition();

        { // Put IPS position in buffer.
            std::lock_guard<std::mutex> lock{arwain::Locks::POSITION_BUFFER_LOCK};
            arwain::Buffers::IPS_BUFFER.pop_front();
            arwain::Buffers::IPS_BUFFER.push_back(position);
        }

        // Log result to file.
        if (arwain::config.log_to_file)
        {
            ips_position_file << time.time_since_epoch().count() << " " << position.x << " " << position.y <<  " " << position.z << "\n";
        }

        // Wait until next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    // Close file handle(s);
    if (arwain::config.log_to_file)
    {
        ips_position_file.close();
    }
}

void arwain::IndoorPositioningWrapper::update(const double &time, const double &x, const double &y, const double &z)
{
    // TODO Pass the parameters to the inner IPS object and put results in
    // m_x, m_y, m_z.
}

vector3 arwain::IndoorPositioningWrapper::getPosition()
{
    return vector3{
        m_x,
        m_y,
        m_z
    };
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
