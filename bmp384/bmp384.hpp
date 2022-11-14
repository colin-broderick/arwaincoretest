#ifndef _BMP384_DRIVER_HPP
#define _BMP384_DRIVER_HPP

#include <tuple>
#include <string>
#include <cstdlib>
#include <sys/ioctl.h>
#include <fcntl.h>
extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

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

// CLASS --------------------------------------------------------------

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

class BMP384
{
    public:
        const static int min_temp = -40;
        const static int max_temp = 85;
        enum class Mode { USE_HYPS, USE_SIMPLE };

    public:
        BMP384() = default;
        BMP384(int bus_address, const std::string &bus_name);
        std::tuple<double, double> read();
        int get_chip_id();
        static double calculate_altitude(const double pressure, const double temperature, const double sea_level_pressure);
        static double hypsometric_altitude(double pressure, double temperature, double sea_level_pressure);
        static double simple_altitude(double pressure, double sea_level_pressure);
        void set_altitude_mode(Mode mode);

    private:
        void set_power_mode(uint8_t mode);
        void i2c_init(const int address, const std::string &bus_name);
        int i2c_read(int reg_addr, int bytes, uint8_t *buffer);
        int i2c_write(int reg_addr, int bytes, uint8_t *buffer);
        void enable_pressure();
        void enable_temperature();
        void set_pressure_osr(uint8_t osr);
        void set_temperature_osr(uint8_t osr);
        void set_iir_filter(uint8_t coefficient);
        void soft_reset();
        double compensate_temperature(uint32_t temperature_uncompensated);
        double compensate_pressure(uint32_t pressure_uncompensated);
        void get_calib_data();

    private:
        int handle = 0;
        const static bool use_hyps = true;
        CalibData calib_data;
};

#endif
