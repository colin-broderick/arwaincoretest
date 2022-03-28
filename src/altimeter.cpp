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

    static const double CONSTANT_ROOM_TEMPERATURE = 21 + 273.15; // We are assuming constant ambient temperature in the hypsometric formula for now.

    // Create pressure sensor.
    BMP384 bmp384{arwain::config.pressure_address, arwain::config.pressure_bus};
    auto [p0, t0] = bmp384.read();
    p0 = p0 - arwain::config.pressure_offset;
    double altitude = BMP384::calculate_altitude(p0 / 100.0, CONSTANT_ROOM_TEMPERATURE, arwain::config.sea_level_pressure);

    // Create a filter to fuse pressure readings and accelerometer readings.
    arwain::Filters::SabatiniAltimeter sabatini_filter{
        altitude,                                                          // Initial altitude.
        0,                                                                 // Initial vertical velocity.
        static_cast<double>(arwain::Intervals::ALTIMETER_INTERVAL)/1000.0, // Time between samples in seconds.
        arwain::config.altimeter_z_accel_stdev,                            // STDEV accelerometer.
        arwain::config.pressure_altitude_stdev                             // STDEV pressure altitude.
    };

    // Few spins to try and get a good starting value for altitude.
    for (int i = 0; i < 20; i++)
    {
        auto [p, t] = bmp384.read();
        p = p - arwain::config.pressure_offset;
        altitude = BMP384::calculate_altitude(p / 100.0, CONSTANT_ROOM_TEMPERATURE, arwain::config.sea_level_pressure);
        altitude = sabatini_filter.update(arwain::rolling_average_accel_z_for_altimeter.get_value() - arwain::config.gravity, altitude);
        sleep_ms(50);
    }
    double altitude_zero = altitude;
    
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
                    altitude = BMP384::calculate_altitude(pressure / 100.0, CONSTANT_ROOM_TEMPERATURE, arwain::config.sea_level_pressure);
                    altitude = sabatini_filter.update(arwain::rolling_average_accel_z_for_altimeter.get_value() - arwain::config.gravity, altitude);
                    {
                        std::lock_guard<std::mutex> lock{arwain::Locks::PRESSURE_BUFFER_LOCK};
                        arwain::Buffers::PRESSURE_BUFFER.pop_front();
                        arwain::Buffers::PRESSURE_BUFFER.push_back({pressure / 100.0, temperature, altitude - altitude_zero});
                    }

                    #ifdef DEBUGALTIMETER
                    std::cout << "Altitude: " << altitude << std::endl;
                    #endif

                    pressure_log << loopTime.time_since_epoch().count() << " " << pressure << " " << temperature << " " << altitude << "\n";

                    // Wait until next tick.
                    loopTime = loopTime + interval;
                    std::this_thread::sleep_until(loopTime);
                }
                break;
            }
            case arwain::OperatingMode::AutoCalibration:
            {
                auto loopTime = std::chrono::system_clock::now();
                std::chrono::milliseconds interval{arwain::Intervals::ALTIMETER_INTERVAL}; // Lower data collection rate in Idle/Autocalibration mode.

                while (arwain::system_mode == arwain::OperatingMode::AutoCalibration)
                {
                    auto [pressure, temperature] = bmp384.read();
                    pressure = pressure - arwain::config.pressure_offset;
                    altitude = BMP384::calculate_altitude(pressure / 100.0, CONSTANT_ROOM_TEMPERATURE, arwain::config.sea_level_pressure);
                    altitude = sabatini_filter.update(arwain::rolling_average_accel_z_for_altimeter.get_value() - arwain::config.gravity, altitude);
                    {
                        std::lock_guard<std::mutex> lock{arwain::Locks::PRESSURE_BUFFER_LOCK};
                        arwain::Buffers::PRESSURE_BUFFER.pop_front();
                        arwain::Buffers::PRESSURE_BUFFER.push_back({pressure / 100.0, temperature, altitude - altitude_zero});
                    }

                    #ifdef DEBUGALTIMETER
                    std::cout << "Altitude: " << altitude << std::endl;
                    #endif

                    // Wait until next tick.
                    loopTime = loopTime + interval;
                    std::this_thread::sleep_until(loopTime);
                }
                altitude_zero = altitude;
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
