#ifndef _BMP384_DRIVER_HPP
#define _BMP384_DRIVER_HPP

#include <tuple>
#include <cmath>
#include <string>
#include <cstdlib>
#include <sstream>
#include <iostream>

#include "i2c_interface.hpp"

// ADDRESSES ------------------------------------------

#define ADDR_CHIP_ID 0x00
#define ADDR_ERR_REG 0x02
#define ADDR_STATUS 0x03
#define ADDR_DATA_0 0x04
#define ADDR_DATA_1 0x05
#define ADDR_DATA_2 0x06
#define ADDR_DATA_3 0x07
#define ADDR_DATA_4 0x08
#define ADDR_DATA_5 0x09
#define ADDR_SENSOR_TIME_0 0x0C
#define ADDR_SENSOR_TIME_1 0x0D
#define ADDR_SENSOR_TIME_2 0x0E
#define ADDR_SENSOR_TIME_3 0x0F
#define ADDR_EVENT 0x10
#define ADDR_INT_STATUS 0x11
#define ADDR_FIFO_LENGTH_0 0x12
#define ADDR_FIFO_LENGTH_1 0x13
#define ADDR_FIFO_DATA 0x14
#define ADDR_FIFO_WTM_0 0x15
#define ADDR_FIFO_WTM_1 0x16
#define ADDR_FIFO_CONFIG_1 0x17
#define ADDR_FIFO_CONFIG_2 0x18
#define ADDR_INT_CTRL 0x19
#define ADDR_IF_CONF 0x1A
#define ADDR_PWR_CTRL 0x1B
#define ADDR_OSR 0x1C
#define ADDR_ODR 0x1D
#define ADDR_CONFIG 0x1F
#define ADDR_CALIB_DATA 0x31
#define ADDR_CMD 0x7E

// COMMANDS -----------------------------------------------------------

#define BMP384_SOFT_RESET 0xB6

// REGISTER MASKS -----------------------------------------------------

#define PWR_MODE_MASK 0b11001111
#define PRESSURE_ENABLE_MASK 0b11111110
#define TEMP_ENABLE_MASK 0b11111101
#define TEMP_OSR_MASK 0b11000111
#define PRESSURE_OSR_MASK 0b11111000
#define IIR_FILTER_MASK 0b11110001

// IIR FILTER COEFFICIENTS --------------------------------------------

#define IIR_COEFF_0 0b00000000
#define IIR_COEFF_1 0b00000010
#define IIR_COEFF_3 0b00000100
#define IIR_COEFF_7 0b00000110
#define IIR_COEFF_15 0b00001000
#define IIR_COEFF_31 0b00001010
#define IIR_COEFF_63 0b00001100
#define IIR_COEFF_127 0b00001110

// OSR VALUES ---------------------------------------------------------

#define PRESSURE_OSR_X1 0b00000000
#define PRESSURE_OSR_X2 0b00000001
#define PRESSURE_OSR_X4 0b00000010
#define PRESSURE_OSR_X8 0b00000011
#define PRESSURE_OSR_X16 0b00000100
#define PRESSURE_OSR_X32 0b00000101

#define TEMP_OSR_X1 0b00000000
#define TEMP_OSR_X2 0b00001000
#define TEMP_OSR_X4 0b00010000
#define TEMP_OSR_X8 0b00011000
#define TEMP_OSR_X16 0b00100000
#define TEMP_OSR_X32 0b00101000

// DEFAULT REGISTER VALUES --------------------------------------------

#define BMP384_CHIP_ID 0x50
#define BMP384_DEFAULT_OSR 0x02
#define BMP384_DEFAULT_ODR 0x00

// MODES --------------------------------------------------------------

#define BMP384_MODE_SLEEP (0x00 << 4)
#define BMP384_MODE_FORCED (0x01 << 4)
#define BMP384_MODE_NORMAL (0x03 << 4)
#define BMP384_PRESSURE_ENABLE 0x01
#define BMP384_PRESSURE_DISABLE 0x00
#define BMP384_TEMP_ENABLE 0x02
#define BMP384_TEMP_DISABLE 0x00

inline void sleep_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds{ms});
}

template <class I2CDeviceClass>
class BMP384
{
    public:
        const static int min_temp = -40;
        const static int max_temp = 85;
        enum class Mode
        {
            USE_HYPS,
            USE_SIMPLE
        };
        struct CalibData
        {
            double t_lin;
            double par_t1;
            double par_t2;
            double par_t3;
            double par_p1;
            double par_p2;
            double par_p3;
            double par_p4;
            double par_p5;
            double par_p6;
            double par_p7;
            double par_p8;
            double par_p9;
            double par_p10;
            double par_p11;
        };

    public:
        BMP384() = default;

        ~BMP384() = default;

        BMP384(int bus_address, const std::string &bus_name)
        {
            if (!transport.i2c_init(bus_address, bus_name))
            {
                std::stringstream ss;
                ss << "Could not connect pressure sensor: ";
                ss << bus_name;
                ss << std::hex;
                ss << " 0x";
                ss << bus_address;
                throw std::runtime_error{ss.str()};
            }

            soft_reset();

            get_calib_data();

            set_pressure_osr(PRESSURE_OSR_X1);
            // set_temperature_osr(TEMP_OSR_X2);
            enable_pressure();
            enable_temperature();
            set_power_mode(BMP384_MODE_NORMAL);
            // set_iir_filter(IIR_COEFF_7);
            sleep_ms(10);
        }

        std::tuple<double, double> read()
        {
            uint8_t bf[6];
            transport.i2c_read(ADDR_DATA_0, 6, bf);

            uint32_t pressure = ((uint32_t)(bf[2]) << 16) | ((uint32_t)(bf[1]) << 8) | ((uint32_t)(bf[0]) >> 0);
            uint32_t temperature = ((uint32_t)(bf[5]) << 16) | ((uint32_t)(bf[4]) << 8) | ((uint32_t)(bf[3]) >> 0);

            double temperature_compensated = compensate_temperature(temperature);
            double pressure_compensated = compensate_pressure(pressure);

            return {pressure_compensated, temperature_compensated};
        }

        int get_chip_id()
        {
            uint8_t id;
            transport.i2c_read(ADDR_CHIP_ID, 1, &id);
            return id;
        }

        /** \brief Compute the altitude using the hypsometric formula.
         * The hypsometric formula considing temperature as a variable. This generally make it more
         * accurate but this may not hold true at extremes of temperature.
         *
         * This function is based on the deriviation by Portland state aerospace society.
         * https://www.researchgate.net/file.PostFileLoader.html?id=5409cac4d5a3f2e81f8b4568&assetKey=AS%3A273593643012096%401442241215893
         *
         * Height h above sea level is given by
         *
         * h = ((P0/P)^(LRg) - 1) * (T/L)    equation (8) in the linked derivation.
         *
         * where P0 = sea level pressure, Pa
         *       P  = measured local pressure, Pa
         *       L  = Lapse rate of temperature with altitude, K/m
         *       R  = gas constant for air, J/kg/K
         *       T  = local measured temperature, Kelvin
         *
         * \param pressure Measured atmospheric pressure in hPa.
         * \param temperature Measured temperature in degrees C.
         * \param sea_level_pressure Current sea level pressure in hPa.
         */
        static double calculate_altitude(const double pressure, const double temperature, const double sea_level_pressure)
        {
            if (use_hyps)
            {
                return hypsometric_altitude(pressure, temperature, sea_level_pressure);
            }
            else
            {
                return simple_altitude(pressure, sea_level_pressure);
            }
        }

        static double hypsometric_altitude(double pressure_hPa, double temperature_K, double sea_level_pressure_hPa)
        {
            const static double lapse_rate = 0.00649; // 0.00649 Kelvin/metre - expected temperature variation per increase in altitude.
            const static double gas_constant_for_air = 287.053;
            const static double gravity = 9.8127;

            return ((std::pow((sea_level_pressure_hPa / pressure_hPa), (lapse_rate * gas_constant_for_air / gravity)) - 1) * (temperature_K)) / lapse_rate;
        }

        static double simple_altitude(double pressure, double sea_level_pressure)
        {
            return 44330.0 * (1.0 - std::pow(pressure / sea_level_pressure, 1.0 / 5.255));
        }
        
        void set_altitude_mode(Mode mode)
        {
            // TODO
            // Will need to make altitude calcualtion functions non-static.
            // if (mode == Mode::USE_HYPS)
            // {
            //     use_hyps = true;
            // }
            // else
            // {
            //     use_hyps = false;
            // }
        }

    TESTABLE:
        void set_power_mode(uint8_t mode)
        {
            uint8_t pwr;
            transport.i2c_read(ADDR_PWR_CTRL, 1, &pwr);
            pwr = (pwr & PWR_MODE_MASK) | mode;
            transport.i2c_write(ADDR_PWR_CTRL, 1, &pwr);
            sleep_ms(2);
        }

        // [[nodiscard]] bool i2c_init(const int address, const std::string &bus_name);
        // int i2c_read(int reg_addr, int bytes, uint8_t *buffer);
        // int i2c_write(int reg_addr, int bytes, uint8_t *buffer);
        void enable_pressure()
        {
            uint8_t reg;
            transport.i2c_read(ADDR_PWR_CTRL, 1, &reg);
            reg = (reg & PRESSURE_ENABLE_MASK) | BMP384_PRESSURE_ENABLE;
            transport.i2c_write(ADDR_PWR_CTRL, 1, &reg);
            sleep_ms(2);
        }

        void enable_temperature()
        {
            uint8_t reg;
            transport.i2c_read(ADDR_PWR_CTRL, 1, &reg);
            reg = (reg & TEMP_ENABLE_MASK) | BMP384_TEMP_ENABLE;
            transport.i2c_write(ADDR_PWR_CTRL, 1, &reg);
            sleep_ms(2);
        }

        void set_pressure_osr(uint8_t osr)
        {
            uint8_t reg;
            transport.i2c_read(ADDR_OSR, 1, &reg);
            reg = (reg & PRESSURE_OSR_MASK) | osr;
            transport.i2c_write(ADDR_OSR, 1, &reg);
            sleep_ms(2);
        }

        void set_temperature_osr(uint8_t osr)
        {
            uint8_t reg;
            transport.i2c_read(ADDR_OSR, 1, &reg);
            reg = (reg & TEMP_OSR_MASK) | osr;
            transport.i2c_write(ADDR_OSR, 1, &reg);
        }

        void set_iir_filter(uint8_t coefficient)
        {
            uint8_t reg;
            transport.i2c_read(ADDR_CONFIG, 1, &reg);
            reg = (reg & IIR_FILTER_MASK) | coefficient;
            transport.i2c_write(ADDR_CONFIG, 1, &reg);
        }

        void soft_reset()
        {
            uint8_t reset = BMP384_SOFT_RESET;
            transport.i2c_write(ADDR_CMD, 1, &reset);
            sleep_ms(2);
        }

        double compensate_temperature(uint32_t temperature_uncompensated)
        {
            double partial_data1 = static_cast<double>(temperature_uncompensated - calib_data.par_t1);
            double partial_data2 = static_cast<double>(partial_data1 * calib_data.par_t2);

            calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data.par_t3;

            if (calib_data.t_lin < BMP384::min_temp)
            {
                calib_data.t_lin = static_cast<double>(BMP384::min_temp);
            }
            if (calib_data.t_lin > BMP384::max_temp)
            {
                calib_data.t_lin = static_cast<double>(BMP384::max_temp);
            }

            return calib_data.t_lin;
        }

        /** \brief Converts the raw temperature reading to something useful.
         * This is not particularly well motivated; the code comes directly from the BMP384 datasheet.
         */
        double compensate_pressure(uint32_t pressure_uncompensated)
        {
            double compensated_pressure, partial_data1, partial_data2, partial_data3, partial_data4, partial_out1, partial_out2;

            partial_data1 = calib_data.par_p6 * calib_data.t_lin;
            partial_data2 = calib_data.par_p7 * calib_data.t_lin * calib_data.t_lin;
            partial_data3 = calib_data.par_p8 * calib_data.t_lin * calib_data.t_lin * calib_data.t_lin;
            partial_out1 = calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

            partial_data1 = calib_data.par_p2 * calib_data.t_lin;
            partial_data2 = calib_data.par_p3 * calib_data.t_lin * calib_data.t_lin;
            partial_data3 = calib_data.par_p4 * calib_data.t_lin * calib_data.t_lin * calib_data.t_lin;
            partial_out2 = static_cast<double>(pressure_uncompensated) * (calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

            partial_data1 = static_cast<double>(pressure_uncompensated) * static_cast<double>(pressure_uncompensated);
            partial_data2 = calib_data.par_p9 + calib_data.par_p10 * calib_data.t_lin;
            partial_data3 = partial_data1 * partial_data2;
            partial_data4 = partial_data3 + static_cast<double>(pressure_uncompensated) * static_cast<double>(pressure_uncompensated) * static_cast<double>(pressure_uncompensated) * calib_data.par_p11;

            compensated_pressure = partial_out1 + partial_out2 + partial_data4;

            return compensated_pressure;
        }

        void get_calib_data()
        {
            uint8_t bf[21];
            transport.i2c_read(ADDR_CALIB_DATA, 21, bf);
            this->calib_data.par_t1 = static_cast<double>(((uint16_t)(bf[1]) << 8) | (uint16_t)(bf[0])) / std::pow(2, -8);
            this->calib_data.par_t2 = static_cast<double>(((uint16_t)(bf[3]) << 8) | (uint16_t)(bf[2])) / std::pow(2, 30);
            this->calib_data.par_t3 = static_cast<double>((int8_t)(bf[4])) / std::pow(2, 48);
            this->calib_data.par_p1 = (static_cast<double>((int16_t)(bf[6] << 8) | (int16_t)(bf[5])) - std::pow(2, 14)) / std::pow(2, 20);
            this->calib_data.par_p2 = (static_cast<double>((int16_t)(bf[8] << 8) | (int16_t)(bf[7])) - std::pow(2, 14)) / std::pow(2, 29);
            this->calib_data.par_p3 = static_cast<double>((int8_t)(bf[9])) / std::pow(2, 32);
            this->calib_data.par_p4 = static_cast<double>((int8_t)(bf[10])) / std::pow(2, 37);
            this->calib_data.par_p5 = static_cast<double>((uint16_t)(bf[12] << 8) | (uint16_t)(bf[11])) / std::pow(2, -3);
            this->calib_data.par_p6 = static_cast<double>((uint16_t)(bf[14] << 8) | (uint16_t)(bf[13])) / std::pow(2, 6);
            this->calib_data.par_p7 = static_cast<double>((int8_t)(bf[15])) / std::pow(2, 8);
            this->calib_data.par_p8 = static_cast<double>((int8_t)(bf[16])) / std::pow(2, 15);
            this->calib_data.par_p9 = static_cast<double>((int16_t)(bf[18] << 8) | (int16_t)(bf[17])) / std::pow(2, 48);
            this->calib_data.par_p10 = static_cast<double>((int8_t)(bf[19])) / std::pow(2, 48);
            this->calib_data.par_p11 = static_cast<double>((int8_t)(bf[20])) / std::pow(2, 65);
            sleep_ms(2);
        }

    TESTABLE:
        I2CDeviceClass transport;
        const static bool use_hyps = true;
        CalibData calib_data;
};

#endif
