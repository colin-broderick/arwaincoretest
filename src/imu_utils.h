#ifndef IMU_UTILS_H
#define IMU_UTILS_H

#include <stdio.h>
#include <string>
#include "bmi2.h"
#include "bmi270.h"
#include "bmm150.h"

typedef struct vec_scaled_output
{
    float x;
    float y;
    float z;
} vec_scaled_output;

int i2c_init(int addr);
int init_bmi270(int mag_enabled, std::string calib_file);
int get_bmi270_data(struct vec_scaled_output *acc, struct vec_scaled_output *gyr);
int get_bmm150_data(struct vec_scaled_output *mag);
void read_calib_data(std::string path);

void delay_us(uint32_t period);
void delay_ms(uint32_t period);
int8_t bmi270_reg_write(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length);
int8_t bmi270_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t bmm150_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t bmm150_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);


#endif
