#include <mutex>

#include "utils.hpp"
#include "bmp280.cpp"
#include "shared_resource.hpp"

/** \brief Uses the BMP280 pressure sensor to determine altitude. */
void altimeter()
{
    // Quit immediately if pressure sensor disabled by configuration.
    if (arwain::config.no_pressure)
    {
        return;
    }

    // Initialize the sensor.
    bmp280_dev bmp;
    bmp280_config conf;
    bmp280_uncomp_data uncomp_data;
    double pres, temp, alt;
    init_bmp280(bmp, conf, uncomp_data, arwain::config.sea_level_pressure);

    // Set up timing.
    auto loopTime = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{250};

    // Spin until shutdown signal received.
    while (!arwain::shutdown)
    {
        // Read the raw data
        bmp280_get_uncomp_data(&uncomp_data, &bmp);

        // Convert the raw data to doubles and calcualte altitude.
        bmp280_get_comp_pres_double(&pres, uncomp_data.uncomp_press, &bmp);
        bmp280_get_comp_temp_double(&temp, uncomp_data.uncomp_temp, &bmp);
        alt = altitude_from_pressure_and_temperature(pres / 100.0, temp);

        // Store output
        {
            std::lock_guard<std::mutex> lock{arwain::Locks::PRESSURE_BUFFER_LOCK};
            arwain::Buffers::PRESSURE_BUFFER.pop_front();
            arwain::Buffers::PRESSURE_BUFFER.push_back({pres / 100.0, temp, alt});
        }

        // Wait until next scheduled run.
        loopTime = loopTime + interval;
        std::this_thread::sleep_until(loopTime);
    }
}
