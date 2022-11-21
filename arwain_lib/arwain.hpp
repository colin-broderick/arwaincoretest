#ifndef _ARWAIN_HPP
#define _ARWAIN_HPP

#include <map>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>

#include "arwain_utils.hpp"
#include "quaternion.hpp"
#include "stance.hpp"
#include "uubla.hpp"
#include "lora.hpp"
#include "configuration.hpp"
#include "global_buffer.hpp"
#include "activity_metric.hpp"

class StanceDetection;

namespace arwain
{
    enum class ReturnCode;
}

arwain::ReturnCode arwain_main(int argc, char** argv);

class Vector3;
class Vector6;
class InputParser;

namespace arwain::BufferSizes
{
    inline const unsigned int POSITION_BUFFER_LEN = 200;
    inline const unsigned int MAG_BUFFER_LEN = 200;
    inline const unsigned int VELOCITY_BUFFER_LEN = 200;
    inline const unsigned int ORIENTATION_BUFFER_LEN = 200;
    inline const unsigned int IMU_BUFFER_LEN = 200;
    inline const unsigned int IPS_BUFFER_LEN = 50;
    inline const unsigned int LORA_MESSAGE_LENGTH = 11;
    inline const unsigned int PRESSURE_BUFFER_LEN = 100;
    inline const unsigned int MAG_ORIENTATION_BUFFER_LEN = 100;
    inline const unsigned int MAG_EULER_BUFFER_LEN = 100;
    
    #if USE_UUBLA
    inline const unsigned int LORA_BEACON_MESSAGE_LENGTH = 2;
    #endif
}

namespace arwain
{
    class Logger;
}

namespace arwain
{
    enum class OperatingMode
    {
        Terminate,
        Inference,
        Idle,
        SelfTest,
        GyroscopeCalibration,
        MagnetometerCalibration,
        AccelerometerCalibration,
        TestSerial,
        TestStanceDetector,
        DataCollection
    };
}

inline std::ostream& operator<<(std::ostream& stream, arwain::OperatingMode token)
{
    switch (token)
    {
        case arwain::OperatingMode::Inference:
            stream << "Inference";
            break;
        case arwain::OperatingMode::Idle:
            stream << "Idle/autocalibrating";
            break;
        case arwain::OperatingMode::SelfTest:
            stream << "Self test";
            break;
        case arwain::OperatingMode::Terminate:
            stream << "Terminate";
            break;
        case arwain::OperatingMode::DataCollection:
            stream << "Data collection";
            break;
        case arwain::OperatingMode::GyroscopeCalibration:
            stream << "Gyroscope calibration";
            break;
        case arwain::OperatingMode::MagnetometerCalibration:
            stream << "Magnetometer calibration";
            break;
        case arwain::OperatingMode::AccelerometerCalibration:
            stream << "Accelerometer calibration";
            break;
        case arwain::OperatingMode::TestSerial:
            stream << "Test serial";
            break;
        case arwain::OperatingMode::TestStanceDetector:
            stream << "Test stance detector";
            break;
        default:
            stream << "Mode not specified";
            break;
    }
    return stream;
}

namespace arwain::Intervals
{
    // Time intervals, all in milliseconds.
    inline const unsigned int IMU_READING_INTERVAL = 5;
    inline const unsigned int VELOCITY_PREDICTION_INTERVAL = 50;
    inline const unsigned int LORA_TRANSMISSION_INTERVAL = 1000;
    inline const unsigned int STANCE_DETECTION_INTERVAL = 1000;
    inline const unsigned int INDOOR_POSITIONING_INTERVAL = 50;
    inline const unsigned int STD_OUT_INTERVAL = 1000;
    inline const unsigned int IPS_INTERVAL = 500;
    inline const unsigned int MAG_READ_INTERVAL = 100;
    inline const unsigned int ALTIMETER_INTERVAL = 20;
}

namespace arwain
{
    extern double yaw_offset;
    extern OperatingMode system_mode;
    extern std::string folder_date_string;
    extern std::string folder_date_string_suffix;
    extern arwain::Configuration config;
    extern arwain::Logger error_log;
    extern unsigned int velocity_inference_rate;
    extern RollingAverage rolling_average_accel_z_for_altimeter;
    extern ActivityMetric activity_metric;

    void setup_log_folder_name_suffix(const InputParser& input);
    arwain::ReturnCode execute_jobs();
    arwain::ReturnCode rerun_orientation_filter(const std::string& data_location);
    arwain::ReturnCode rerun_floor_tracker(const std::string& data_location);
    void setup_log_directory();
    arwain::ReturnCode calibrate_gyroscopes();
    arwain::ReturnCode calibrate_accelerometers();
    arwain::ReturnCode calibrate_accelerometers_simple();
    arwain::ReturnCode calibrate_magnetometers();

    const std::string help_text = "Run without arguments for no logging\n"
        "\n"
        "Arguments:\n"
        "    -lstd          Log friendly output to stdout\n"
        "    -conf          Specify alternate configuration file\n"
        "    -calib         Perform online calibration - make sure the device is totally stationary\n"
        "    -noinf         Do not do velocity inference\n"
        "    -noimu         Do not turn on the IMU - for testing\n"
        "    -nolora        Do not attempt to enable LoRa chip or send transmissions\n"
        "    -nopressure    Do not use the pressure sensor to assist altitude tracking\n"
        "    -h             Show this help text\n"
        "  (The following arguments are exclusive)\n"
        "    -testimu       Sends IMU data (a,g) to stdout - other flags are ignored if this is set\n"
        "    -testori       Runs an orientation filter against IMU 1 and prints results as quaternion and euler triplet\n"
        "    -calibg        Calibrate the gyroscope for a give IMU. Must specify -bus and -address.\n"
        "    -caliba        Calibrate the acceleration for a given IMU. Must specify -bus and -address.\n"
        "                   If a configuration file is specified, the result of the calibrations will be written there.\n"
        "        -bus       The bus on which to find the IMU, e.g. /dev/i2c-1\n"
        "        -address   The address of the IMU in hexadecimal, e.g. 0x68\n"
        "\n"
        "Example usages:\n"
        "    ./arwain -lstd -calib calib.txt -conf arwain.conf\n"
        "    ./arwain -calibg -bus /dev/i2c-1 -address 0x68 -conf /etc/arwain.conf\n"
        "\n"
        "Return codes:\n"
        "     1             Successfully executed\n"
        "    -1             IMU failed to start\n"
        "    -2             Problem reading configuration file\n"
        "    -3             Inference model XML not found";
}

namespace arwain::Buffers
{
    extern GlobalBuffer<Vector6, arwain::BufferSizes::IMU_BUFFER_LEN> IMU_BUFFER;
    extern GlobalBuffer<Vector6, arwain::BufferSizes::IMU_BUFFER_LEN> IMU_WORLD_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::VELOCITY_BUFFER_LEN> VELOCITY_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::POSITION_BUFFER_LEN> POSITION_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::MAG_BUFFER_LEN> MAG_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::MAG_BUFFER_LEN> MAG_WORLD_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::IPS_BUFFER_LEN> IPS_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::PRESSURE_BUFFER_LEN> PRESSURE_BUFFER;
    extern GlobalBuffer<EulerOrientation, arwain::BufferSizes::ORIENTATION_BUFFER_LEN> EULER_ORIENTATION_BUFFER;
    extern GlobalBuffer<Quaternion, arwain::BufferSizes::ORIENTATION_BUFFER_LEN> QUAT_ORIENTATION_BUFFER;
    extern GlobalBuffer<Quaternion, arwain::BufferSizes::MAG_ORIENTATION_BUFFER_LEN> MAG_ORIENTATION_BUFFER;
    extern GlobalBuffer<double, arwain::BufferSizes::MAG_EULER_BUFFER_LEN> MAG_EULER_BUFFER;
}

#endif
