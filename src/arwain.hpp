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

class vector3;
class vector6;
class quaternion;
class InputParser;

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
    extern int shutdown;
    extern std::string folder_date_string;
    extern arwain::Configuration config;
    extern arwain::Status status;
    extern arwain::Logger error_log;
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
    extern std::deque<vector6> IMU_BUFFER;
    extern std::deque<vector6> IMU_WORLD_BUFFER;
    extern std::deque<vector3> VELOCITY_BUFFER;
    extern std::deque<vector3> POSITION_BUFFER;
    extern std::deque<vector3> MAG_BUFFER;
    extern std::deque<vector3> MAG_WORLD_BUFFER;
    extern std::deque<vector3> IPS_BUFFER;
    extern std::deque<vector3> PRESSURE_BUFFER;
    extern std::deque<euler_orientation_t> EULER_ORIENTATION_BUFFER;
    extern std::deque<quaternion> QUAT_ORIENTATION_BUFFER;
    extern std::deque<quaternion> MAG_ORIENTATION_BUFFER;
}

namespace arwain
{
    void setup(const InputParser& input);
    int test_imu();
    int test_lora_tx();
    int test_lora_rx();
    int test_mag();
    int test_ori(int rate);
    int execute_inference();
}

namespace arwain::ExitCodes
{
    inline const int Success = 0;
    inline const int FailedIMU = -1;
    inline const int FailedConfiguration = -2;
    inline const int InferenceXMLMissing = -3;
    inline const int FailedMagnetometer = -4;
}

namespace arwain::Intervals
{
    // Time intervals, all in milliseconds.
    inline const unsigned int IMU_READING_INTERVAL = 5;
    inline const unsigned int VELOCITY_PREDICTION_INTERVAL = 50;
    inline const unsigned int LORA_TRANSMISSION_INTERVAL = 500;
    inline const unsigned int STANCE_DETECTION_INTERVAL = 1000;
    inline const unsigned int INDOOR_POSITIONING_INTERVAL = 50;
    inline const unsigned int STD_OUT_INTERVAL = 1000;
    inline const unsigned int IPS_INTERVAL = 500;
    inline const unsigned int MAG_READ_INTERVAL = 100;
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
}

namespace arwain::SystemStates
{
    enum _SystemStates
    {
        Inference,
        GyroscopeCalibration,
        AccelerometerCalibration,
        HelpText
    };
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

    int calibrate_gyroscopes();
    int calibrate_accelerometers();
    int calibrate_magnetometers();

    /** \brief Configuration struct for whole programme. */
    class Configuration
    {
        public:
            Configuration(){};
            Configuration(const InputParser& input);
            int read_from_file();

        public:
            double active_threshold; // Heuristic parameter used to distinguish types of motion at similar speeds. NOT YET WELL DEFINED.
            double walking_threshold; // Velocity while vertical above which a subject is inferred to be walking.
            double running_threshold; // Velocity while vertical above which a subject is inferred to be running.
            double crawling_threshold; // Velocity while horizontal above which a subject is inferred to be crawling.
            double climbing_threshold; // Vertical velocity above which the activity is inferred to be climbing.
            double gravity; // Magnitude of local gravity, e.g. 9.81.
            double struggle_threshold; // Heuristic parameter used to determine when a subject may be in distress. NOT YET WELL DEFINED.
            double freefall_sensitivity; // The sensitivity to freefall detection.
            vector3 accel1_bias; // The systematic bias in measurements from accelerometer 1.
            vector3 accel2_bias; // The systematic bias in measurements from accelerometer 2.
            vector3 accel3_bias; // The systematic bias in measurements from accelerometer 3.
            vector3 gyro1_bias; // The systematic bias in measurements from gyroscope 1.
            vector3 gyro2_bias; // The systematic bias in measurements from gyroscope 2.
            vector3 gyro3_bias; // The systematic bias in measurements from gyroscope 3.
            vector3 mag_bias; // Biases in magnetometer measurements, subtracted from readings before processing.
            vector3 mag_scale; // Magnetometer scale factor, multipled by readings before processing.
            int use_magnetometer; // Whether to use the magnetometer for orientation filtering.
            int log_magnetometer; // Whether to take and log magnetometer readings.
            double npu_vel_weight_confidence; // Relative confidence in NPU vs. IMU integration for velocity predictions, 1 being 100% NPU, 0 being 100% integration.
            double madgwick_beta; // Madgwick filter gain parameter.
            double efaroe_beta; // EFAROE filter gain parameter.
            double efaroe_zeta; // EFAROE filter gain parameter.
            int use_indoor_positioning_system; // Whether to use IPS for stair and floor snapping.
            std::string orientation_filter; // Which orientation filter to use out of options [efaroe, madgwick].
            LoRa::Frequency lora_rf_frequency; // Frequency in MHz of the LoRa radio.
            int lora_packet_frequency; // Times per second to transmit LoRa packet.
            int lora_tx_power; // LoRa transmission power.
            LoRa::SpreadFactor lora_spread_factor; // LoRa spread factor.
            LoRa::Bandwidth lora_bandwidth; // LoRa bandwidth.
            LoRa::CodingRate lora_coding_rate; // LoRa coding rate.
            std::string lora_address; // spidev address of SX1276.
            int lora_sync_word; // LoRa sync word.
            LoRa::HeaderMode lora_header_mode; // LoRa header mode, implicit or explicit.
            int lora_enable_crc; //  Whether to add CRC to LoRa messages.
            std::string inference_model_xml; // The location of the inference model xml file.
            double sea_level_pressure; // Sea level pressure near the region of interest.
            int log_to_stdout = 0; // Whether to show printed output on the console.
            int log_to_file = 0; // Whether to log data to file.
            int no_inference = 0; // Disables inference using NPU.
            int no_imu = 0; // Disables IMU.
            int no_lora = 0; // Disables LoRa radio.
            int no_pressure = 0; // Disables pressure sensor.
            std::string config_file = "/etc/arwain.conf"; // Location of configuration file.
            std::string imu1_bus; // The I2C bus on which to find IMU1.
            std::string imu2_bus; // The I2C bus on which to find IMU2.
            std::string imu3_bus; // The I2C bus on which to find IMU3.
            int imu1_address; // The I2C address of IMU1.
            int imu2_address; // The I2C address of IMU2.
            int imu3_address; // The I2C address of IMU3.
            std::string magn_bus; // The I2C bus on which to find the magnetometer.
            int magn_address; // The I2C address of the magnetometer.
            std::string pressure_bus; // The I2C bus of the pressure sensor.
            int pressure_address; // The I2C address of the pressure sensor.
            int node_id; // A unique ID number for the node.

            /** \brief Overwrite the content of the configuration file associated with this struct.
             * \param option The configuration option to overwrite.
             * \param new_value The new new value of the parameter to be put in the file.
             */
            template<typename T> void replace(const std::string& option, const T new_value)
            {
                std::ifstream infile(this->config_file);
                std::stringstream outstring;

                std::string line;
                while (std::getline(infile, line))
                {
                    auto delimiter = line.find("=");
                    std::string name = line.substr(0, delimiter);
                    if (name == option)
                    {
                        outstring << name << "=" << std::setprecision(15) << new_value << "\n";
                    }
                    else
                    {
                        outstring << line << "\n";
                    }
                }
                infile.close();

                std::ofstream outfile(this->config_file);
                outfile << outstring.str();
                outfile.close();
            }

            /** \brief Reads an option into the specified storage space in the configuration struct.
             * \param option String naming the option to parse from the file.
             * \param storage The storage location of the data read from file, typically a struct member.
             * \return ARWAIN exist code indiciating success or error.
             */
            template<typename T> int read_option(const std::map<std::string, std::string>& options, const std::string& option, T& storage)
            {
                // TODO Add error checking for bad reads and send it up the call stack.
                std::stringstream(options.at(option)) >> storage;
                return arwain::ExitCodes::Success;
            }
    };
}

namespace arwain
{
    const std::string help_text = "Run without arguments for no logging\n"
        "\n"
        "Arguments:\n"
        "    -lstd          Log friendly output to stdout\n"
        "    -lfile         Record sensor data to file - files are stored in ./data_<datetime>\n"
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
        "    ./arwain -lstd -calib calib.txt -conf arwain.conf -lfile\n"
        "    ./arwain -calibg -bus /dev/i2c-1 -address 0x68 -conf /etc/arwain.conf\n"
        "\n"
        "Return codes:\n"
        "     1             Successfully executed\n"
        "    -1             IMU failed to start\n"
        "    -2             Problem reading configuration file\n"
        "    -3             Inference model XML not found";
}

#endif
