#ifndef _ARWAIN_ALTIMETER_HPP
#define _ARWAIN_ALTIMETER_HPP

#include <tuple>

#include "arwain/sabatini_altimeter.hpp"
#include "arwain/logger.hpp"
#include "arwain/job_interface.hpp"
#include "arwain/i2c_interface.hpp"

#include <arwain/devices/bmp384.hpp>

class Altimeter : public ArwainJob, protected IArwainJobSpec
{
    private:
        void run() override;
        void core_setup() override;
        void run_inference() override;
        void run_idle() override;
        void setup_inference() override;
        bool cleanup_inference() override;

    private:
        double altitude;
        double altitude_zero;
        const double CONSTANT_ROOM_TEMPERATURE = 21 + 273.15; // We are assuming constant ambient temperature in the hypsometric formula for now.

        std::jthread job_thread;
        arwain::Logger pressure_log;
        BMP384<PlatformI2CDevice> bmp384;
        arwain::Filters::SabatiniAltimeter sabatini_filter;

    public:
        Altimeter();
        bool join();
};

#endif
