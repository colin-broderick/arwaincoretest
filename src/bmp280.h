#ifndef BMP280_H
#define BMP280_H

#include <math.h>

namespace arwain
{
    class BMP280
    {
        public:
            // Constructors
            BMP280();

            // Getters
            double getTemperature();
            double getPressure();
            double getAltitude();

        private:
            // Configuration
            static unsigned char t_mode;
            static unsigned char p_mode;
            static unsigned char standby;
            static unsigned char filter;

            // Registers addresses and values.
            //// BMP280 i2c address.
            static unsigned char I2CADDR = 0x76;
            //// Operating modes.
            static unsigned char OSAMPLE_1 = 1;
            static unsigned char OSAMPLE_2 = 2;
            static unsigned char OSAMPLE_4 = 3;
            static unsigned char OSAMPLE_8 = 4;
            static unsigned char OSAMPLE_16 = 5;
            //// Standby settings.
            static unsigned char STANDBY_0p5 = 0;
            static unsigned char STANDBY_62p5 = 1;
            static unsigned char STANDBY_125 = 2;
            static unsigned char STANDBY_250 = 3;
            static unsigned char STANDBY_500 = 4;
            static unsigned char STANDBY_1000 = 5;
            static unsigned char STANDBY_10 = 6;
            static unsigned char STANDBY_20 = 7;
            //// Filter configuration.
            static unsigned char FILTER_off = 0;
            static unsigned char FILTER_2 = 1;
            static unsigned char FILTER_4 = 2;
            static unsigned char FILTER_8 = 3;
            static unsigned char FILTER_16 = 4;
            //// Registers.
            static unsigned char REGISTER_T1 = 0x88;
            static unsigned char REGISTER_T2 = 0x8A;
            static unsigned char REGISTER_T3 = 0x8C;
            static unsigned char REGISTER_P1 = 0x8E;
            static unsigned char REGISTER_P2 = 0x90;
            static unsigned char REGISTER_P3 = 0x92;
            static unsigned char REGISTER_P4 = 0x94;
            static unsigned char REGISTER_P5 = 0x96;
            static unsigned char REGISTER_P6 = 0x98;
            static unsigned char REGISTER_P7 = 0x9A;
            static unsigned char REGISTER_P8 = 0x9C;
            static unsigned char REGISTER_P9 = 0x9E;
            static unsigned char REGISTER_CHIPID = 0xD0;
            static unsigned char REGISTER_VERSION = 0xD1;
            static unsigned char REGISTER_SOFTRESET = 0xE0;
            static unsigned char REGISTER_STATUS = 0xF3;
            static unsigned char REGISTER_CONTROL = 0xF4;
            static unsigned char REGISTER_CONFIG = 0xF5;
            static unsigned char REGISTER_DATA = 0xF7;

            // Calibration values
            static unsigned int p1;
            static unsigned int p2;
            static unsigned int p3;
            static unsigned int t1;
            static unsigned int t2;
            static unsigned int t3;
            static unsigned int t4;
            static unsigned int t5;
            static unsigned int t6;
            static unsigned int t7;
            static unsigned int t8;
            static unsigned int t9;

            bool gotPressure;
            bool gotTemperature;
            bool gotAltitude;

            double altitude;
            double pressure;
            double temperature;
            double sea_level_pressure;

            bool hypsometric;

            void calculateAltitude();
            void load_calibration();

            unsigned char read_register(unsigned char address, unsigned int num_bytes);
            void write_register(unsigned char address);
    };
}

#endif
