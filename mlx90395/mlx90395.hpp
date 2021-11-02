#ifndef _GREEVE_MLX90395_HPP
#define _GREEVE_MLX90395_HPP

#include <iostream>
#include <map>
#include <array>
#include <sys/ioctl.h>
#include <fcntl.h>
extern "C"
{
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

#include "vector3.hpp"

// CONSTANTS ===================================================================

#define GRAVITY 9.8067
#define PI_DIVIDED_BY_180 0.01745329251994329576923690768489

// CONFIG_MASKS ================================================================
#define MAG_OSR_1 0
#define MAG_OSR_2 1
#define MAG_OSR_4 2
#define MAG_OSR_8 3

#define MAG_DIF_FILT_0 0b00000000_00000000
#define MAG_DIF_FILT_0 0b00000000_00000000
#define MAG_DIF_FILT_0 0b00000000_00000000
#define MAG_DIF_FILT_0 0b00000000_00000000
#define MAG_DIF_FILT_0 0b00000000_00000000
#define MAG_DIF_FILT_0 0b00000000_00000000
#define MAG_DIF_FILT_0 0b00000000_00000000
#define MAG_DIF_FILT_0 0b00000000_00000000

#define TMP_CMP_EN 0b00000100_00000000
#define TMP_CMP_DS 0b00000000_00000000

#define CMD_RESET 0xF0
#define CMD_EXIT 0x80

// DATA REGISTERS ================================================================
#define CONFIGURATION_01 0x01
#define CONFIGURATION_02 0x02
#define CMD_TARGET 0x80
#define READ_TARGET 0x80

// DATA CONVERSION FACTORS =========================================================

class MLX90395
{
public:
    MLX90395(int bus_address, const std::string &bus_name);
    vector3 read();
    vector3 calibrate();

private:
    bool configure();
    bool reset();
    bool exit_mode();
    void send_read_command();
    uint8_t get_gain();
    uint8_t get_resolution();
    void set_osr(uint8_t);
    void set_resolution(uint8_t);
    void i2c_init(const int address, const std::string &bus_name);
    int i2c_read(int reg_addr, int bytes, uint8_t *buffer);
    int i2c_write(int reg_addr, int bytes, uint8_t *buffer);

private:
    float mag_x, mag_y, mag_z;
    int handle = 0;
    double mag_gain;       // todo check data type
    double mag_resolution; // todo check data type
    double uTLSB;          // todo check data type
    const std::array<float, 16> gain_multipliers = {
        0.2, 0.25, 0.3333, 0.4, 0.5, 0.6, 0.75, 1,
        0.1, 0.125, 0.1667, 0.2, 0.25, 0.3, 0.375, 0.5};
};

/** \brief Compute the mean value of a 1-dimensional ArrayType of doubles.
 * \param[in] data An iterable ArrayType of 1 dimension containing doubles.
 * \return The mean of the double values in the data array.
 */
template <typename ArrayType>
static double array_mean_1d(const ArrayType &data)
{
    double total = 0;
    for (const double &element : data)
    {
        total += element;
    }
    return total / data.size();
}

#endif
