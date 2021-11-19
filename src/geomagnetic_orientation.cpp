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

    arwain::Logger mag_log;

    if (arwain::config.log_to_file)
    {
        mag_log.open(arwain::folder_date_string + "/multi_quat.txt");
        mag_log << "time x y z\n";
    }

    auto time = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds interval{arwain::Intervals::MAG_READ_INTERVAL};

    while (!arwain::shutdown)
    {
        // TODO the task
        vector3 reading = magnetometer.read();
        quaternion orientation = magnetometer.read_orientation();

        if (arwain::config.log_to_file)
        {
            mag_log << reading.x << " " << reading.y << " " << reading.z << "\n";
        }

        time += interval;
        std::this_thread::sleep_until(time);
    }

    if (arwain::config.log_to_file)
    {
        mag_log.close();
    }
}