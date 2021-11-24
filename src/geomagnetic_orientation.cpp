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

    arwain::Logger mag_log, mag_ori_log;

    if (arwain::config.log_to_file)
    {
        mag_log.open(arwain::folder_date_string + "/mag.txt");
        mag_ori_log.open(arwain::folder_date_string + "/mag_orientation.txt");
        mag_log << "time x y z\n";
        mag_ori_log << "time w x y z\n";
    }

    auto time = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds interval{arwain::Intervals::MAG_READ_INTERVAL};

    while (!arwain::shutdown)
    {
        auto timeCount = time.time_since_epoch().count();

        vector3 reading = magnetometer.read();
        quaternion orientation = magnetometer.read_orientation();

        {
            std::lock_guard<std::mutex> lock{arwain::Locks::MAG_BUFFER_LOCK};
            arwain::Buffers::MAG_BUFFER.push_back(reading);
            arwain::Buffers::MAG_ORIENTATION_BUFFER.push_back(orientation);
        }

        std::cout << std::atan2(reading.z, reading.y)*180.0/3.14159 << std::endl;

        if (arwain::config.log_to_file)
        {
            mag_log << timeCount << " " << reading.x << " " << reading.y << " " << reading.z << "\n";
            mag_ori_log << timeCount << " " << orientation.w << " " << orientation.x << " " << orientation.y << " " << orientation.z << "\n";
        }

        time += interval;
        std::this_thread::sleep_until(time);
    }

    if (arwain::config.log_to_file)
    {
        mag_log.close();
        mag_ori_log.close();
    }
}
