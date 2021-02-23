#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <sstream>
#include <ctime>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <map>

std::chrono::_V2::system_clock::time_point now();

typedef struct euler_orientation_t
{
    double roll;
    double pitch;
    double yaw;
} euler_orientation_t;

// Date/time as a string in the form YYYY_MM_DD_HH_mm_ss.
std::string datetimestring();

void test_imu();

struct configuration {
    double active_threshold;
    double walking_threshold;
    double running_threshold;
    double crawling_threshold;
    double climbing_threshold;
    double gravity;
    double struggle_threshold;
    double a_threshold;
    double accel_bias_x;
    double accel_bias_y;
    double accel_bias_z;
    double gyro_bias_x;
    double gyro_bias_y;
    double gyro_bias_z;
    double mag_bias_x;
    double mag_bias_y;
    double mag_bias_z;
    double mag_scale_x;
    double mag_scale_y;
    double mag_scale_z;
    int use_magnetometer;
    int log_magnetometer;
    double npu_vel_weight_confidence;
    double madgwick_beta;
    double efaroe_beta;
    double efaroe_zeta;
};

configuration get_configuration(std::string filename);

#endif
