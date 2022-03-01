#include <mutex>
#include <thread>

#include "bmp384.hpp"
#include "arwain.hpp"
#include "logger.hpp"
#include "sabatini_altimeter.hpp"

/** \brief Uses the BMP384 pressure sensor to determine altitude. */
void altimeter()
{
    // Quit immediately if pressure sensor disabled by configuration.
    if (arwain::config.no_pressure)
    {
        return;
    }

    // Create pressure sensor, then average altitude over 1 second to get baseline.
    BMP384 bmp384{arwain::config.pressure_address, arwain::config.pressure_bus};
    double altitude = 0;
    for (int i = 0; i < 20; i++)
    {
        auto [p, t] = bmp384.read();
        p = p - arwain::config.pressure_offset;
        double new_altitude = BMP384::calculate_altitude(p / 100.0, t, arwain::config.sea_level_pressure);
        altitude = (altitude * i + new_altitude) / (double)(i + 1);
        sleep_ms(50);
    }

    // Create a filter to fuse pressure readings and accelerometer readings.
    arwain::Filters::SabatiniAltimeter sabatini_filter{
        altitude,                                                          // Initial altitude.
        0,                                                                 // Initial vertical velocity.
        static_cast<double>(arwain::Intervals::ALTIMETER_INTERVAL)/1000.0, // Time between samples in seconds.
        0.2,                                                               // STDEV accelerometer.
        2                                                                  // STDEV pressure altitude.
    };
    
    // Spin until shutdown signal received.
    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::Inference:
            {
                // Set up logging and timing.
                arwain::Logger pressure_log;
                pressure_log.open(arwain::folder_date_string + "/pressure.txt");
                pressure_log << "time pressure temperature altitude\n";

                auto loopTime = std::chrono::system_clock::now();
                std::chrono::milliseconds interval{arwain::Intervals::ALTIMETER_INTERVAL};

                while (arwain::system_mode == arwain::OperatingMode::Inference)
                {
                    auto [pressure, temperature] = bmp384.read();
                    pressure = pressure - arwain::config.pressure_offset;
                    double new_altitude = BMP384::calculate_altitude(pressure / 100.0, temperature, arwain::config.sea_level_pressure);
                    altitude = sabatini_filter.update(arwain::Buffers::IMU_WORLD_BUFFER.back().acce.z - arwain::config.gravity, new_altitude);
                    {
                        std::lock_guard<std::mutex> lock{arwain::Locks::PRESSURE_BUFFER_LOCK};
                        arwain::Buffers::PRESSURE_BUFFER.pop_front();
                        arwain::Buffers::PRESSURE_BUFFER.push_back({pressure / 100.0, temperature, altitude});
                    }

                    pressure_log << loopTime.time_since_epoch().count() << " " << pressure << " " << temperature << " " << altitude << "\n";

                    // Wait until next tick.
                    loopTime = loopTime + interval;
                    std::this_thread::sleep_until(loopTime);
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
