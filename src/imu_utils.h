#ifndef IMU_UTILS_H
#define IMU_UTILS_H

#include <stdio.h>
#include <string>
#include <vector>

#include "bmi2.h"
#include "bmi270.h"
#include "bmm150.h"
#include "vector3.h"

int i2c_init(const int address, int& file_i2c);
int init_bmi270(int mag_enabled, std::string calib_file);
int get_bmi270_data(struct vector3 *acc, struct vector3 *gyr);
int get_bmm150_data(struct vector3 *mag);
void read_calib_data(std::string path);
float get_bmi270_temperature();


void delay_us(uint32_t period);
void delay_ms(uint32_t period);
int8_t bmi270_reg_write(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length);
int8_t bmi270_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t bmm150_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t bmm150_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

#endif
