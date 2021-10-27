#ifndef GREEVE_ARWAIN_UTILS_HPP
#define GREEVE_ARWAIN_UTILS_HPP

#include <string>
#include <sstream>
#include <ctime>
#include <fstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <string>
#include <map>
#include <thread>
#include <pthread.h>

#include "vector3.hpp"
#include "stance.hpp"
#include "lora.hpp"
#include "input_parser.hpp"
#include "IMU_IIM42652_driver.hpp"

namespace arwain
{
    const std::string help_text = "Run without arguments for no logging\n"
        "\n"
        "Arguments:\n"
        "  -lstd        Log friendly output to stdout\n"
        "  -lfile       Record sensor data to file - files are stored in ./data_<datetime>\n"
        "  -conf        Specify alternate configuration file\n"
        "  -calib       Perform online calibration - make sure the device is totally stationary\n"
        "  -testimu     Sends IMU data (a,g) to stdout - other flags are ignored if this is set\n"
        "  -noinf       Do not do velocity inference\n"
        "  -noimu       Do not turn on the IMU - for testing\n"
        "  -nolora      Do not attempt to enable LoRa chip or send transmissions\n"
        "  -nopressure  Do not use the pressure sensor to assist altitude tracking\n"
        "  -h           Show this help text\n"
        "\n"
        "Example usage:\n"
        "  ./arwain -lstd -calib calib.txt -conf arwain.conf -lfile\n"
        "\n"
        "Error codes:\n"
        "   1           Successfully executed\n"
        "  -1           IMU failed to start\n"
        "  -2           Problem reading configuration file";
}

namespace arwain::ExitCodes
{
    static const int Success = 1;
    static const int FailedIMU = -1;
    static const int FailedConfiguration = -2;
}

namespace arwain::Intervals
{
    // Time intervals, all in milliseconds.
    static const unsigned int IMU_READING_INTERVAL = 5;
    static const unsigned int VELOCITY_PREDICTION_INTERVAL = 50;
    static const unsigned int LORA_TRANSMISSION_INTERVAL = 500;
    static const unsigned int STANCE_DETECTION_INTERVAL = 1000;
    static const unsigned int INDOOR_POSITIONING_INTERVAL = 50;
    static const unsigned int STD_OUT_INTERVAL = 1000;
}

namespace arwain::BufferSizes
{
    static const unsigned int POSITION_BUFFER_LEN = 200;
    static const unsigned int MAG_BUFFER_LEN = 200;
    static const unsigned int VELOCITY_BUFFER_LEN = 200;
    static const unsigned int ORIENTATION_BUFFER_LEN = 200;
    static const unsigned int IMU_BUFFER_LEN = 200;
    static const unsigned int IPS_BUFFER_LEN = 50;
    static const unsigned int LORA_MESSAGE_LENGTH = 8;
    static const unsigned int PRESSURE_BUFFER_LEN = 100;
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
     * \param lora_rf_frequency Frequency in MHz of the LoRa radio.
     * \param lora_packet_frequency Times per second to transmit LoRa packet.
     * \param lora_tx_power LoRa transmission power.
     * \param lora_spread_factor LoRa spread factor.
     * \param lora_bandwidth LoRa bandwidth.
     * \param lora_coding_rate LoRa coding rate.
     * \param lora_sync_word LoRa sync word.
     * \param lora_header_mode LoRa header mode, implicit or explicit.
     * \param lora_enable_crc Whether to add CRC to LoRa messages.
     * \param inference_model_xml The location of the inference model xml file.
     * \param sea_level_pressure Sea level pressure near the region of interest.
     */
    class Configuration
    {
        public:
            Configuration(){};
            Configuration(const InputParser& input);

        private:
            int read_from_file();

        public:
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
            LoRa::freq_t lora_rf_frequency;
            int lora_packet_frequency;
            int lora_tx_power;
            LoRa::sf_t lora_spread_factor;
            LoRa::bw_t lora_bandwidth;
            LoRa::cr_t lora_coding_rate;
            int lora_sync_word;
            LoRa::hm_t lora_header_mode;
            int lora_enable_crc;
            std::string inference_model_xml;
            double sea_level_pressure;
            int log_to_stdout = 0;
            int log_to_file = 0;
            int no_inference = 0;
            int no_imu = 0;
            int no_lora = 0;
            int no_pressure = 0;
            std::string config_file = "/etc/arwain.conf";
            int file_read_ok;

            /** \brief Overwrite the content of the configuration file associated with this struct.
             * \param option The configuration option to overwrite.
             * \param new_value The new new value of the parameter to be put in the file.
             */
            template<typename T> void replace(const std::string& option, const T new_value)
            {
                std::ifstream infile(this->config_file);
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

                std::ofstream outfile(this->config_file);
                outfile << outstring.str();
                outfile.close();
            }
    };

    std::string datetimestring();

    struct Status {
        arwain::StanceDetector::Stance current_stance;
        arwain::StanceDetector::Attitude attitude;
        arwain::StanceDetector::EntangleState entangled;
        arwain::StanceDetector::FallState falling;
        arwain::Errors::ErrorCondition errors;
        int IMUTemperature;
    };
    
    void test_imu();
    float getCPUTemp();
}

typedef struct euler_orientation_t
{
    double roll;
    double pitch;
    double yaw;
} euler_orientation_t;

#endif
