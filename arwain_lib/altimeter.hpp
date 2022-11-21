#ifndef _ARWAIN_ALTIMETER_HPP
#define _ARWAIN_ALTIMETER_HPP

#include <tuple>

#include "arwain.hpp"
#include "sabatini_altimeter.hpp"
#include "bmp384.hpp"

class Altimeter
{
    TESTABLE:
        void run();
        void core_setup();
        void run_inference();
        void run_idle();
        void pause();
        void setup_inference();
        void cleanup_inference();

    private:
        double altitude;
        double altitude_zero;
        const double CONSTANT_ROOM_TEMPERATURE = 21 + 273.15; // We are assuming constant ambient temperature in the hypsometric formula for now.

        ArwainThread job_thread;
        arwain::Logger pressure_log;
        BMP384 bmp384;
        arwain::Filters::SabatiniAltimeter sabatini_filter;

    public:
        Altimeter();
        bool init();
        void join();
};

#endif
