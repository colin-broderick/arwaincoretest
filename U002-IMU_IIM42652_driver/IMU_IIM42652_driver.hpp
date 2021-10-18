#ifndef GREEVE_IIM42652_HPP
#define GREEVE_IIM42652_HPP


#include <iostream>
#include <map>
#include <sys/ioctl.h>
#include <fcntl.h>
extern "C"
{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

// CONSTANTS ===================================================================

#define GRAVITY 9.8067
#define PI_DIVIDED_BY_180 0.01745329251994329576923690768489

// CONFIG_MASKS ================================================================

#define CONFIG_SOFT_RESET     0b00000001
#define CONFIG_ENABLE_ACCEL   0b00000011
#define CONFIG_ENABLE_GYRO    0b00001100
#define CONFIG_ENABLE_TMPRTR  0b00000000
#define CONFIG_DISABLE_TMPRTR 0b01000000

#define ACCEL_ODR_200HZ   0b00000111
#define ACCEL_FSR_2G  0b01100000
#define ACCEL_FSR_16G 0b00000000

#define GYRO_ODR_200HZ    0b00000111
#define GYRO_FSR_2000 0b00000000

// DATA REGISTERS ================================================================

#define ADDR_DEVICE_CONFIG 0x11
#define ADDR_PWR_MGMT0 0x4E

#define ADDR_ACCEL_CONFIG0 0x50
#define ADDR_GYRO_CONFIG0 0x4F

#define ADDR_TMPRTR 0x1D

#define ADDR_ALL_IMU 0x1F
#define ADDR_ACCEL_X1 0x1F
#define ADDR_ACCEL_X0 0x20
#define ADDR_ACCEL_Y1 0x21
#define ADDR_ACCEL_Y0 0x22
#define ADDR_ACCEL_Z1 0x23
#define ADDR_ACCEL_Z0 0x24

#define ADDR_GYRO_X1 0x25
#define ADDR_GYRO_X0 0x26
#define ADDR_GYRO_Y1 0x27
#define ADDR_GYRO_Y0 0x28
#define ADDR_GYRO_Z1 0x29
#define ADDR_GYRO_Z0 0x2A

// DATA CONVERSION FACTORS =========================================================

#define ACCEL_RES_2G 0.00006103515625
#define ACCEL_RES_4G 0.0001220703125
#define ACCEL_RES_8G 0.000244140625
#define ACCEL_RES_16G 0.00048828125

#define GYRO_RES_15_625 4.76837158203125e-4
#define GYRO_RES_31_25 9.5367431640625e-4
#define GYRO_RES_62_5 0.0019073486328125
#define GYRO_RES_125 0.003814697265625
#define GYRO_RES_250 0.00762939453125
#define GYRO_RES_500 0.0152587890625
#define GYRO_RES_1000 0.030517578125
#define GYRO_RES_2000 0.06103515625

class IMU_IIM42652
{
public:
    IMU_IIM42652(int bus_address, std::string bus_name);
    void soft_reset();
    int IMU_config(uint8_t gypro_config, uint8_t accl_config);
    void set_resolutions(double accel, double gyro);
    void read_IMU();
    double read_temperature();

private:
    double accel_resolution = 0;
    double gyro_resolution = 0;
    int handle = 0;

private:
    void i2c_init(const int address, std::string bus_name);
    void enable();
    int i2c_read(int reg_addr, int bytes, uint8_t *buffer);
    int i2c_write(int reg_addr, int bytes, uint8_t *buffer);
    void read_IMU_raw_data();

public:
    double temperature = 0;
    double gyroscope_x = 0;
    double gyroscope_y = 0;
    double gyroscope_z = 0;
    double accelerometer_x = 0;
    double accelerometer_y = 0;
    double accelerometer_z = 0;
};

#endif
