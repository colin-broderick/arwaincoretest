#include <thread>
#include <chrono>

#include "logger.hpp"
#include "arwain.hpp"
#include "lis3mdl.hpp"

void mag_reader()
{
    if (arwain::config.log_magnetometer)
    {
        return;
    }

    LIS3MDL magnetometer{arwain::config.magn_address, arwain::config.magn_bus};
    magnetometer.set_calibration(arwain::config.mag_bias, arwain::config.mag_scale);

    arwain::Logger mag_log, mag_game_rv_log, mag_euler_log;

    if (arwain::config.log_to_file)
    {
        mag_log.open(arwain::folder_date_string + "/mag.txt");
        mag_game_rv_log.open(arwain::folder_date_string + "/mag_game_rv.txt");
        mag_euler_log.open(arwain::folder_date_string + "/mag_euler_orientation.txt");
        mag_log << "time x y z\n";
        mag_euler_log << "time yaw\n";
        mag_game_rv_log << "time w x y z\n";
    }

    auto time = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds interval{arwain::Intervals::MAG_READ_INTERVAL};

    while (!arwain::shutdown)
    {
        auto timeCount = time.time_since_epoch().count();

        Vector3 reading = magnetometer.read();
        quaternion orientation = magnetometer.read_orientation();
        double yaw = std::atan2(reading.y, reading.z);

        {
            std::lock_guard<std::mutex> lock{arwain::Locks::MAG_BUFFER_LOCK};
            arwain::Buffers::MAG_BUFFER.push_back(reading);
            arwain::Buffers::MAG_BUFFER.pop_front();
            arwain::Buffers::MAG_ORIENTATION_BUFFER.push_back(orientation);
            arwain::Buffers::MAG_ORIENTATION_BUFFER.pop_front();
            arwain::Buffers::MAG_EULER_BUFFER.push_back(yaw);
            arwain::Buffers::MAG_EULER_BUFFER.pop_front();
        }

        if (arwain::config.log_to_file)
        {
            mag_log << timeCount << " " << reading.x << " " << reading.y << " " << reading.z << "\n";
            mag_game_rv_log << timeCount << " " << orientation.w << " " << orientation.x << " " << orientation.y << " " << orientation.z << "\n";
            mag_euler_log << timeCount << " " << yaw << "\n";
        }

        time += interval;
        std::this_thread::sleep_until(time);
    }

    if (arwain::config.log_to_file)
    {
        mag_log.close();
        mag_game_rv_log.close();
        mag_euler_log.close();
    }
}
