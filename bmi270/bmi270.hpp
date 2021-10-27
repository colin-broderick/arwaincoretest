#ifndef GREEVE_BMI270_HPP
#define GREEVE_BMI270_HPP

#include <string>
#include <unistd.h>

#include "bmi2_defs.h"
#include "bmi270.h"
#include "bmm150_defs.h"
#include "bmm150.h"

#include "vector3.hpp"

int i2c_init(const int address, int& file_i2c, const std::string& i2c_bus);
int init_bmi270(int mag_enabled, const std::string& calib_file, const std::string& i2c_bus);
int8_t bmi270_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t bmi270_reg_write(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length);
int8_t bmm150_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t bmm150_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int get_bmi270_data(struct vector3 *acc, struct vector3 *gyr);
void delay_us(uint32_t period);
void delay_ms(uint32_t period);

vector3 mag_calib_offset, mag_calib_scale;

void read_calib_data(const std::string& path);

int bmi_file_i2c = -1;

double acc_scale;
double gyr_scale;

struct bmi2_dev bmi270;
struct bmm150_dev bmm150;

struct bmi2_sensor_data acce;
struct bmi2_sensor_data gyro;
struct bmi2_sensor_data magn;

class BMI270
{
    public:
        BMI270(const int i2c_address, const std::string& i2c_bus);
        double read_temperature();
        void read_IMU();

    public:
        vector3 acce, gyro;
        double accelerometer_x;
        double accelerometer_y;
        double accelerometer_z;
        double gyroscope_x;
        double gyroscope_y;
        double gyroscope_z;
};

#endif
