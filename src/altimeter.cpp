#include <mutex>

#include "bmp280.cpp"
#include "bmp384.hpp"
#include "arwain.hpp"

/** \brief Uses the BMP384 pressure sensor to determine altitude. */
void altimeter()
{
    // Quit immediately if pressure sensor disabled by configuration.
    if (arwain::config.no_pressure)
    {
        return;
    }

    BMP384 bmp384{arwain::config.pressure_address, arwain::config.pressure_bus};

    // Set up timing.
    auto loopTime = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{250};

    // Spin until shutdown signal received.
    while (!arwain::shutdown)
    {
        auto [pressure, temperature] = bmp384.read();
        double alt = BMP384::calculate_altitude(pressure / 100.0, temperature, arwain::config.sea_level_pressure);
        {
            std::lock_guard<std::mutex> lock{arwain::Locks::PRESSURE_BUFFER_LOCK};
            arwain::Buffers::PRESSURE_BUFFER.pop_front();
            arwain::Buffers::PRESSURE_BUFFER.push_back({pressure / 100.0, temperature, alt});
        }
        // Wait until next tick.
        loopTime = loopTime + interval;
        std::this_thread::sleep_until(loopTime);
    }
}
