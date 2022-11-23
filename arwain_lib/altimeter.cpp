#include <mutex>
#include <thread>

#include "bmp384.hpp"
#include "arwain.hpp"
#include "logger.hpp"
#include "arwain_thread.hpp"
#include "exceptions.hpp"
#include "sabatini_altimeter.hpp"
#include "altimeter.hpp"
#include "arwain_utils.hpp"
#include "timers.hpp"

Altimeter::Altimeter()
{
    init();
}

void Altimeter::setup_inference()
{
    pressure_log.open(arwain::folder_date_string + "/pressure.txt");
    pressure_log << "time pressure temperature altitude\n";
}

void Altimeter::cleanup_inference()
{
    pressure_log.close();
}

void Altimeter::run_inference()
{
    setup_inference();

    Timers::IntervalTimer<std::chrono::milliseconds> loop_scheduler{arwain::Intervals::ALTIMETER_INTERVAL, "altimeter_infer"};

    while (arwain::system_mode == arwain::OperatingMode::Inference)
    {
        auto [pressure, temperature] = bmp384.read();
        pressure = pressure - arwain::config.pressure_offset;
        altitude = BMP384::calculate_altitude(pressure / 100.0, CONSTANT_ROOM_TEMPERATURE, arwain::config.sea_level_pressure);
        altitude = sabatini_filter.update(arwain::rolling_average_accel_z_for_altimeter.get_value() - arwain::config.gravity, altitude);
        arwain::Buffers::PRESSURE_BUFFER.push_back({pressure / 100.0, temperature, altitude - altitude_zero});

        pressure_log << loop_scheduler.count() << " " << pressure << " " << temperature << " " << altitude << "\n";

        // Wait until next tick.
        loop_scheduler.await();
    }

    cleanup_inference();
}

void Altimeter::run_idle()
{
    Timers::IntervalTimer<std::chrono::milliseconds> loop_scheduler{arwain::Intervals::ALTIMETER_INTERVAL, "altimeter_idle"};

    while (arwain::system_mode == arwain::OperatingMode::Idle)
    {
        auto [pressure, temperature] = bmp384.read();
        pressure = pressure - arwain::config.pressure_offset;
        altitude = BMP384::calculate_altitude(pressure / 100.0, CONSTANT_ROOM_TEMPERATURE, arwain::config.sea_level_pressure);
        altitude = sabatini_filter.update(arwain::rolling_average_accel_z_for_altimeter.get_value() - arwain::config.gravity, altitude);
        arwain::Buffers::PRESSURE_BUFFER.push_back({pressure / 100.0, temperature, altitude - altitude_zero});

        // Wait until next tick.
        loop_scheduler.await();
    }
    altitude_zero = altitude;
}

void Altimeter::run()
{
    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::Inference:
                run_inference();
                break;
            case arwain::OperatingMode::Idle:
                run_idle();
                break;
            default:
                sleep_ms(10);
                break;
        }
    }
}

void Altimeter::core_setup()
{
    bmp384 = BMP384{arwain::config.pressure_address, arwain::config.pressure_bus};
    auto [p0, t0] = bmp384.read();
    p0 = p0 - arwain::config.pressure_offset;
    altitude = BMP384::calculate_altitude(p0 / 100.0, CONSTANT_ROOM_TEMPERATURE, arwain::config.sea_level_pressure);
    sabatini_filter = arwain::Filters::SabatiniAltimeter{
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
    altitude_zero = altitude;
}

bool Altimeter::init()
{
    if (arwain::config.no_pressure)
    {
        return false;
    }
    core_setup();
    job_thread = ArwainThread{&Altimeter::run, "arwain_altr_th", this};
    return true;
}

void Altimeter::join()
{
    if (job_thread.joinable())
    {
        job_thread.join();
    }
}
