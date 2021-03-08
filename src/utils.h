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

/** \brief Configuration struct for whole programme.
 * \param active_threshold Heuristic parameter used to distinguish types of motion at similar speeds. NOT YET WELL DEFINED.
 * \param walking_threshold Velocity while vertical above which a subject is inferred to be walking.
 * \param running_threshold  Velocity while vertical above which a subject is inferred to be running.
 * \param crawling_threshold Velocity while horizontal above which a subject is inferred to be crawling.
 * \param climbing_threshold Vertical velocity above which the activity is inferred to be climbing.
 * \param gravity Magnitude of local gravity, e.g. 9.81.
 * \param struggle_threshold Heuristic parameter used to determine when a subject may be in distress. NOT YET WELL DEFINED.
 * \param freefall_sensitivity The acceleration magnitude below which a freefall event will be triggered.
 * \param accel_bias Biases in accelerometer measurements, subtracted from readings before processing.
 * \param gyro_bias Biases in gyroscope measurements, subtracted from readings before processing.
 * \param mag_bias Biases in magnetometer measurements, subtracted from readings before processing.
 * \param mag_scale Magnetometer scale factor, multipled by readings before processing.
 * \param use_magnetometer Whether to use the magnetometer for orientation filtering.
 * \param log_magnetometer Whether to take and log magnetometer readings.
 * \param npu_vel_weight_confidence Relative confidence in NPU vs. IMU integration for velocity predictions, 1 being 100% NPU, 0 being 100% integration.
 * \param madgwick_beta Madgwick filter gain parameter.
 * \param efaroe_beta EFAROE filter gain parameter.
 * \param efaroe_zeta EFAROE filter gain parameter.
 * \param use_indoor_positioning_system Whether to use IPS for stair and floor snapping.
 * \param orientation_filter Which orientation filter to use out of options [efaroe, madgwick].
 */
struct Configuration {
    double active_threshold;
    double walking_threshold;
    double running_threshold;
    double crawling_threshold;
    double climbing_threshold;
    double gravity;
    double struggle_threshold;
    double freefall_sensitivity;
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
    int use_indoor_positioning_system;
    std::string orientation_filter;
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
        std::string name = line.substr(0, delimiter);
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
