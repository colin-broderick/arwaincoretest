#ifndef GREEVE_IIM42652_HPP
#define GREEVE_IIM42652_HPP

#include <iostream>
#include <map>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <eigen3/Eigen/Dense>
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

#define CONFIG_SOFT_RESET     0b00000001
#define CONFIG_ENABLE_ACCEL   0b00000011
#define CONFIG_ENABLE_GYRO    0b00001100
#define CONFIG_ENABLE_TMPRTR  0b00000000
#define CONFIG_DISABLE_TMPRTR 0b01000000

#define ACCEL_200HZ  0b00000111
#define ACCEL_FSR_2G 0b01100000
#define ACCEL_FSR_16G 0b00000000

#define GYRO_200HZ      0b00000111
#define GYRO_FSR_2000   0b00000000
#define GYRO_FSR_1000   0b00100000
#define GYRO_FSR_500    0b01000000
#define GYRO_FSR_250    0b01100000
#define GYRO_FSR_125    0b10000000
#define GYRO_FSR_62_5   0b10100000
#define GYRO_FSR_31_25  0b11000000
#define GYRO_FSR_15_625 0b11100000

#define GYRO_UI_FILTER_ORD_3RD 0b00001000

// DATA REGISTERS ================================================================

#define ADDR_DEVICE_CONFIG 0x11
#define ADDR_PWR_MGMT0 0x4E

#define ADDR_ACCEL_CONFIG0 0x50
#define ADDR_GYRO_CONFIG0 0x4F
#define ADDR_GYRO_CONFIG1 0x51

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
    IMU_IIM42652(int bus_address, const std::string& bus_name);
    void soft_reset();
    int IMU_config(uint8_t gyro_config, uint8_t accel_config);
    void set_resolutions(double accel, double gyro);
    Vector6 read_IMU();
    double read_temperature();
    Vector3 calibrate_gyroscope();
    Vector3 calibration_accel_sample();
    void set_gyro_bias(double x, double y, double z);
    void set_accel_bias(double x, double y, double z);
    void set_accel_scale(double x, double y, double z);
    void set_correction_speed(double speed);
    void enable_auto_calib();
    void enable_auto_calib(double threshold);
    void disable_auto_calib();
    Vector3 get_gyro_calib();
    double get_gyro_calib_x();
    double get_gyro_calib_y();
    double get_gyro_calib_z();

private:
    double accel_resolution = 0;
    double gyro_resolution = 0;
    int handle = 0;
    double temperature = 0;

    // Calibration parameters.
    double gyro_bias_x = 0;
    double gyro_bias_y = 0;
    double gyro_bias_z = 0;
    double accel_bias_x = 0;
    double accel_bias_y = 0;
    double accel_bias_z = 0;
    double accel_scale_x = 1;
    double accel_scale_y = 1;
    double accel_scale_z = 1;

    // Most recent reading lives here.
    double gyroscope_x = 0;
    double gyroscope_y = 0;
    double gyroscope_z = 0;
    double accelerometer_x = 0;
    double accelerometer_y = 0;
    double accelerometer_z = 0;
    
    double auto_calib_timer = 0;
    int calib_time = 200;
    double auto_calib_threshold = 0.025;
    bool auto_calib_enabled = false;
    double correction_speed = 0.995;

private:
    void i2c_init(const int address, const std::string& bus_name);
    void enable();
    int i2c_read(int reg_addr, int bytes, uint8_t *buffer);
    int i2c_write(int reg_addr, int bytes, uint8_t *buffer);
    void read_IMU_raw_data();
    void update_gyro_bias();
};

/** \brief Compute the mean value of a 1-dimensional ArrayType of doubles.
 * \param[in] data An iterable ArrayType of 1 dimension containing doubles.
 * \return The mean of the double values in the data array.
 */
template<typename ArrayType>
static double array_mean_1d(const ArrayType& data)
{
    double total = 0;
    for (const double& element : data)
    {
        total += element;
    }
    return total / data.size();
}

#endif
