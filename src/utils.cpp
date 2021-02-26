#include <sstream>
#include <thread>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "utils.h"
#include "imu_utils.h"

// Get current time.
std::chrono::_V2::system_clock::time_point now()
{
    return std::chrono::system_clock::now();
}

std::string datetimestring()
{
    std::time_t now = std::time(0);
    std::tm *ltm = localtime(&now);
    std::stringstream ss;

    // Year
    ss << ltm->tm_year+1900;
    ss << "_";

    // Month
    if (ltm->tm_mon+1 < 10)
    {
        ss << '0' << ltm->tm_mon+1;
    }
    else
    {
        ss << ltm->tm_mon+1;
    }
    ss << "_";

    // Date
    if (ltm->tm_mday < 10)
    {
        ss << '0' << ltm->tm_mday;
    }
    else
    {
        ss << ltm->tm_mday;
    }
    ss << "_";
    
    // Hour
    if (ltm->tm_hour < 10)
    {
        ss << '0' << ltm->tm_hour;
    }
    else
    {
        ss << ltm->tm_hour;
    }
    ss << "_";

    // Minute
    if (ltm->tm_min < 10)
    {
        ss << '0' << ltm->tm_min;
    }
    else
    {
        ss << ltm->tm_min;
    }
    ss << "_";

    // Second
    if (ltm->tm_sec < 10)
    {
        ss << '0' << ltm->tm_sec;
    }
    else
    {
        ss << ltm->tm_sec;
    }

    return ss.str();
}

void test_imu(int &shutdown)
{
    // Initialize the IMU.
    std::string path = "../calib.txt";
    if (init_bmi270(0, path) != 0)
    {
        printf("Node failed to start\n");
        exit(1);
    }

    // Local buffers for IMU data
    vector3 accel_data;
    vector3 gyro_data;

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(5);

    while (!shutdown)
    {
        get_bmi270_data(&accel_data, &gyro_data);

        // Display IMU data.
        std::cout << time.time_since_epoch().count() << std::fixed << std::right << std::setprecision(3) << "\t" << accel_data.x << "\t" << accel_data.y << "\t" << accel_data.z << "\t" << gyro_data.x << "\t" << gyro_data.y << "\t" << gyro_data.z << "\n";

        // Wait until the next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);

    }
}

Configuration get_configuration(std::string filename)
{
    using std::string;
    using std::stringstream;

    // Open the configuration file name.
    std::ifstream file(filename);

    // A map to store key value pairs from the configuration file.
    std::map<string, string> options;

    // Read each line into the map, based on the format "key=value".
    string line;
    while (getline(file, line))
    {
        if (line[0] == '[' || line.empty())
        {
            continue;
        }
        auto delimiter = line.find("=");
        auto name = line.substr(0, delimiter);
        auto value = line.substr(delimiter + 1);
        options[name] = value;
    }

    // TODO Detect attempted read of non-existing options.

    // Read all options into a configuration object.
    Configuration cf;
    stringstream(options["active_threshold"]) >> cf.active_threshold;
    stringstream(options["walking_threshold"]) >> cf.walking_threshold;
    stringstream(options["running_threshold"]) >> cf.running_threshold;
    stringstream(options["crawling_threshold"]) >> cf.crawling_threshold;
    stringstream(options["climbing_threshold"]) >> cf.climbing_threshold;
    stringstream(options["gravity"]) >> cf.gravity;
    stringstream(options["struggle_threshold"]) >> cf.struggle_threshold;
    stringstream(options["fall_threshold"]) >> cf.fall_threshold;
    stringstream(options["accel_bias_x"]) >> cf.accel_bias.x;
    stringstream(options["accel_bias_y"]) >> cf.accel_bias.y;
    stringstream(options["accel_bias_z"]) >> cf.accel_bias.z;
    stringstream(options["gyro_bias_x"]) >> cf.gyro_bias.x;
    stringstream(options["gyro_bias_y"]) >> cf.gyro_bias.y;
    stringstream(options["gyro_bias_z"]) >> cf.gyro_bias.z;
    stringstream(options["mag_bias_x"]) >> cf.mag_bias.x;
    stringstream(options["mag_bias_y"]) >> cf.mag_bias.y;
    stringstream(options["mag_bias_z"]) >> cf.mag_bias.z;
    stringstream(options["mag_scale_x"]) >> cf.mag_scale.x;
    stringstream(options["mag_scale_y"]) >> cf.mag_scale.y;
    stringstream(options["mag_scale_z"]) >> cf.mag_scale.z;
    stringstream(options["use_magnetometer"]) >> cf.use_magnetometer;
    stringstream(options["log_magnetometer"]) >> cf.log_magnetometer;
    stringstream(options["npu_vel_weight_confidence"]) >> cf.npu_vel_weight_confidence;
    stringstream(options["madgwick_beta"]) >> cf.madgwick_beta;
    return cf;
}
