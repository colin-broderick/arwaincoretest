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
#include "quaternion.hpp"

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
#define CMD_BURST 0x1E
#define CMD_BURST_WITH_T 0x1F
#define CMD_READ 0x40
#define CMD_EXIT 0x80

// DATA REGISTERS ================================================================
#define REGISTER_0 0x00
#define REGISTER_1 0x02
#define REGISTER_2 0x04

#define CMD_TARGET 0x80
#define READ_TARGET 0x80
#define STATUS_RESET 0x02

// DATA CONVERSION FACTORS =========================================================

class MLX90395
{
public:
    MLX90395(int bus_address, const std::string &bus_name);
    vector3 read();
    quaternion read_orientation();
    vector3 calibrate();

    enum OSR
    {
        OSR_1 = 0,
        OSR_2,
        OSR_4,
        OSR_8
    };

    enum Resolution
    {
        RES_16 = 0,
        RES_17,
        RES_18,
        RES_19
    };

private:
    bool configure();
    bool reset();
    bool exit_mode();
    uint8_t burst_mode();
    void send_read_command();
    bool set_data_rate(uint8_t rate);
    uint8_t command(uint8_t cmd);
    uint8_t get_gain();
    uint8_t get_resolution();
    void set_gain(uint8_t gain_);

    void set_osr(OSR osr_val);
    void set_resolution(Resolution res);
    void i2c_init(const int address, const std::string &bus_name);
    int i2c_read(int reg_addr, int bytes, uint8_t *buffer);
    int i2c_write(int reg_addr, int bytes, uint8_t *buffer);

private:
    float mag_x = 0;
    float mag_y = 0;
    float mag_z = 0;
    int handle = 0;
    uint8_t mag_gain;       // todo check data type
    uint8_t mag_resolution; // todo check data type
    double uTLSB;          // todo check data type
    
};

#endif
