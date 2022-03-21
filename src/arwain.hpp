#ifndef _ARWAIN_HPP
#define _ARWAIN_HPP

#include <map>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>

#include "stance.hpp"
#include "lora.hpp"
#include "configuration.hpp"

void sleep_ms(int ms);

class Vector3;
class Vector6;
class Quaternion;
class InputParser;

/** \brief Computes a true rolling average as values are fed in.
 * 
 * Averages will be produced and can be obtained before the window is filled,
 * although this will likely not be a useful value before the averaging window
 * is filled. The method .ready() can be called to confirm that the roller has
 * been fed enough values to fill the window and the average value is therefore
 * valid.
 */
class RollingAverage
{
    public:
        RollingAverage(unsigned int window_size_);
        bool ready();
        void feed(double value);
        double get_value();

    private:
        unsigned int window_size;
        double current_average = 0;
        std::deque<double> stack;
};

double unwrap_phase_degrees(double new_angle, double previous_angle);
double unwrap_phase_radians(double new_angle, double previous_angle);

struct euler_orientation_t
{
    double roll;
    double pitch;
    double yaw;
};

namespace arwain
{
    class Configuration;
    class Status;
    class Logger;
}

namespace arwain
{
    enum class OperatingMode
    {
        Terminate,
        Inference,
        AutoCalibration,
        SelfTest,
        GyroscopeCalibration,
        MagnetometerCalibration,
        AccelerometerCalibration,
        TestSerial,
        TestStanceDetector
    };
}

inline std::ostream& operator<<(std::ostream& stream, arwain::OperatingMode token)
{
    switch (token)
    {
        case arwain::OperatingMode::Inference:
            stream << "Inference";
            break;
        case arwain::OperatingMode::AutoCalibration:
            stream << "Idle/autocalibrating";
            break;
        case arwain::OperatingMode::SelfTest:
            stream << "Self test";
            break;
        case arwain::OperatingMode::Terminate:
            stream << "Terminate";
            break;
        default:
            stream << "Mode not specified";
            break;
    }
    return stream;
}

namespace arwain
{
    extern double yaw_offset;
    extern OperatingMode system_mode;
    extern std::string folder_date_string;
    extern std::string folder_date_string_suffix;
    extern arwain::Configuration config;
    extern arwain::Status status;
    extern arwain::Logger error_log;
    extern bool request_gyro_calib;
    extern bool ready_for_inference;
    extern unsigned int velocity_inference_rate;
    extern RollingAverage rolling_average_accel_z_for_altimeter;
}

/** \brief Contains mutex locks for thread coordination. */
namespace arwain::Locks
{
    extern std::mutex PRESSURE_BUFFER_LOCK;
    extern std::mutex IMU_BUFFER_LOCK;
    extern std::mutex MAG_BUFFER_LOCK;
    extern std::mutex VELOCITY_BUFFER_LOCK;
    extern std::mutex STATUS_FLAG_LOCK;
    extern std::mutex POSITION_BUFFER_LOCK;
    extern std::mutex ORIENTATION_BUFFER_LOCK;
    extern std::mutex PRESSURE_BUFFER_LOCK;
}

namespace arwain::Buffers
{
    extern std::deque<Vector6> IMU_BUFFER;
    extern std::deque<Vector6> IMU_WORLD_BUFFER;
    extern std::deque<Vector3> VELOCITY_BUFFER;
    extern std::deque<Vector3> POSITION_BUFFER;
    extern std::deque<Vector3> MAG_BUFFER;
    extern std::deque<Vector3> MAG_WORLD_BUFFER;
    extern std::deque<Vector3> IPS_BUFFER;
    extern std::deque<Vector3> PRESSURE_BUFFER;
    extern std::deque<euler_orientation_t> EULER_ORIENTATION_BUFFER;
    extern std::deque<Quaternion> QUAT_ORIENTATION_BUFFER;
    extern std::deque<Quaternion> MAG_ORIENTATION_BUFFER;
    extern std::deque<double> MAG_EULER_BUFFER;
}

namespace arwain
{
    void setup(const InputParser& input);
    int execute_inference();
    int rerun_orientation_filter(const std::string& data_location);
    int rerun_floor_tracker(const std::string& data_location);
}

namespace arwain::ExitCodes
{
    inline const int Success = 0;
    inline const int FailedIMU = -1;
    inline const int FailedConfiguration = -2;
    inline const int InferenceXMLMissing = -3;
    inline const int FailedMagnetometer = -4;
}

namespace arwain::ExitCodes::Calibration
{
    inline const int CalibrationApplied = -5;
    inline const int CalibrationNotApplied = -6;
    inline const int NotEnoughData = -7;
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

namespace arwain::BufferSizes
{
    inline const unsigned int POSITION_BUFFER_LEN = 200;
    inline const unsigned int MAG_BUFFER_LEN = 200;
    inline const unsigned int VELOCITY_BUFFER_LEN = 200;
    inline const unsigned int ORIENTATION_BUFFER_LEN = 200;
    inline const unsigned int IMU_BUFFER_LEN = 200;
    inline const unsigned int IPS_BUFFER_LEN = 50;
    inline const unsigned int LORA_MESSAGE_LENGTH = 9;
    inline const unsigned int PRESSURE_BUFFER_LEN = 100;
    inline const unsigned int MAG_ORIENTATION_BUFFER_LEN = 100;
    inline const unsigned int MAG_EULER_BUFFER_LEN = 100;
}

namespace arwain::Errors
{
    enum class ErrorCondition
    {
        AllOk,
        IMUReadError,
        OtherError
    };
}

namespace arwain
{
    inline std::map<int, std::string> ErrorMessages = {
        {arwain::ExitCodes::InferenceXMLMissing, "Inference model XML file not found."},
        {arwain::ExitCodes::FailedIMU, "Could not communicate with IMU."},
        {arwain::ExitCodes::FailedConfiguration, "Configuration file not found."},
    };
}

namespace arwain
{
    struct Status
    {
        arwain::StanceDetector::Stance current_stance;
        arwain::StanceDetector::Attitude attitude;
        arwain::StanceDetector::EntangleState entangled;
        arwain::StanceDetector::FallState falling;
        arwain::Errors::ErrorCondition errors;
        int IMUTemperature;
    };
}

namespace arwain
{
    std::string datetimestring();
    float getPiCPUTemp();
    void setup_log_directory();
    int calibrate_gyroscopes();
    int calibrate_accelerometers();
    int calibrate_accelerometers_simple();
    int calibrate_magnetometers();
}

namespace arwain
{
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

#endif
