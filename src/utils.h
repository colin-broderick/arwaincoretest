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

#include "vector3.h"
#include "stance.h"

std::chrono::_V2::system_clock::time_point now();

typedef struct euler_orientation_t
{
    double roll;
    double pitch;
    double yaw;
} euler_orientation_t;

std::string datetimestring();

void test_imu(int &shutdown);

struct Status {
    StanceDetector::STANCE current_stance;
    StanceDetector::ATTITUDE attitude;
    StanceDetector::ENTANGLED entangled;
    StanceDetector::FALLING falling;
};

struct Configuration {
    double active_threshold;
    double walking_threshold;
    double running_threshold;
    double crawling_threshold;
    double climbing_threshold;
    double gravity;
    double struggle_threshold;
    double fall_threshold;
    vector3 accel_bias;
    vector3 gyro_bias;
    vector3 mag_bias;
    vector3 mag_scale;
    int use_magnetometer;
    int log_magnetometer;
    double npu_vel_weight_confidence;
    double madgwick_beta;
    double efaroe_beta;
    double efaroe_zeta;
};

Configuration get_configuration(const std::string &filename);

/** \brief Overwrite the content of a configuration file.
 * \param filename The file in which to do the write operation.
 * \param option The configuration option to overwrite.
 * \param new_value The new value of the parameter to be put in the file.
 */
template <class T>
void config_replace(std::string filename, std::string option, T new_value)
{
    std::ifstream infile(filename);
    std::stringstream outstring;

    std::string line;
    while (getline(infile, line))
    {
        auto delimiter = line.find("=");
        auto name = line.substr(0, delimiter);
        if (name == option)
        {
            outstring << name << "=" << new_value << "\n";
        }
        else
        {
            outstring << line << "\n";
        }
    }
    infile.close();

    std::ofstream outfile(filename);
    outfile << outstring.str();
    outfile.close();
}

#endif
