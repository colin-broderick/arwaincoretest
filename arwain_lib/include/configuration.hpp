#ifndef _ARWAIN_CONFIG_HPP
#define _ARWAIN_CONFIG_HPP

#include <map>
#include <string>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <filesystem>

#include "input_parser.hpp"
#include "vector3.hpp"
#include "lora.hpp"
#include "arwain_utils.hpp"

/** \brief Configuration struct for whole programme. */
namespace arwain
{
    class Configuration
    {
    public:
        Configuration(){};
        Configuration(const InputParser &input);
        arwain::ReturnCode read_from_file();

    public:
        double active_threshold;     // Heuristic parameter used to distinguish types of motion at similar speeds. NOT YET WELL DEFINED.
        double walking_threshold;    // Velocity while vertical above which a subject is inferred to be walking.
        double running_threshold;    // Velocity while vertical above which a subject is inferred to be running.
        double crawling_threshold;   // Velocity while horizontal above which a subject is inferred to be crawling.
        double climbing_threshold;   // Vertical velocity above which the activity is inferred to be climbing.
        double gravity;              // Magnitude of local gravity, e.g. 9.81.
        double struggle_threshold;   // Heuristic parameter used to determine when a subject may be in distress. NOT YET WELL DEFINED.
        double freefall_sensitivity; // The sensitivity to freefall detection.
        Vector3 accel1_bias;         // The systematic bias in measurements from accelerometer 1.
        Vector3 accel2_bias;         // The systematic bias in measurements from accelerometer 2.
        Vector3 accel3_bias;         // The systematic bias in measurements from accelerometer 3.
        Vector3 accel1_scale;
        Vector3 accel2_scale;
        Vector3 accel3_scale;
        Vector3 gyro1_bias; // The systematic bias in measurements from gyroscope 1.
        Vector3 gyro2_bias; // The systematic bias in measurements from gyroscope 2.
        Vector3 gyro3_bias; // The systematic bias in measurements from gyroscope 3.
        Vector3 mag_bias;   // Biases in magnetometer measurements, subtracted from readings before processing.
        Vector3 mag_scale;  // Magnetometer scale factor, multipled by readings before processing.
        double mag_scale_xy;
        double mag_scale_xz;
        double mag_scale_yz;
        int use_magnetometer;                         // Whether to use the magnetometer for orientation filtering.
        int log_magnetometer;                         // Whether to take and log magnetometer readings.
        int use_ips;                                  // Whether to use the indoor positioning system (floor levelling, corner detection, etc).
        double npu_vel_weight_confidence;             // Relative confidence in NPU vs. IMU integration for velocity predictions, 1 being 100% NPU, 0 being 100% integration.
        double madgwick_beta;                         // Madgwick filter gain parameter.
        double madgwick_beta_conv;                    // Madgwick gain while magnetmometer converges.
        double efaroe_beta;                           // EFAROE filter gain parameter.
        double efaroe_zeta;                           // EFAROE filter gain parameter.
        std::string orientation_filter;               // Which orientation filter to use out of options [efaroe, madgwick].
        LoRa::Frequency lora_rf_frequency;            // Frequency in MHz of the LoRa radio.
        int lora_packet_frequency;                    // Times per second to transmit LoRa packet.
        int lora_tx_power;                            // LoRa transmission power.
        LoRa::SpreadFactor lora_spread_factor;        // LoRa spread factor.
        LoRa::Bandwidth lora_bandwidth;               // LoRa bandwidth.
        LoRa::CodingRate lora_coding_rate;            // LoRa coding rate.
        std::string lora_address;                     // spidev address of SX1276.
        int lora_sync_word;                           // LoRa sync word.
        LoRa::HeaderMode lora_header_mode;            // LoRa header mode, implicit or explicit.
        int lora_enable_crc;                          //  Whether to add CRC to LoRa messages.
        std::string inference_model_xml;              // The location of the inference model xml file.
        double sea_level_pressure;                    // Sea level pressure near the region of interest.
        int log_to_stdout = 0;                        // Whether to show printed output on the console.
        int no_inference = 0;                         // Disables inference using NPU.
        int no_imu = 0;                               // Disables IMU.
        int no_lora = 0;                              // Disables LoRa radio.
        int no_pressure = 0;                          // Disables pressure sensor.
        int no_cli = 0;                               // Disable thie built-in command-line interface.
        int no_stance = 0;                            // Disable stance detection subsystem.
        std::string config_file = "/etc/arwain.conf"; // Location of configuration file.
        std::string imu1_bus;                         // The I2C bus on which to find IMU1.
        std::string imu2_bus;                         // The I2C bus on which to find IMU2.
        std::string imu3_bus;                         // The I2C bus on which to find IMU3.
        int imu1_address;                             // The I2C address of IMU1.
        int imu2_address;                             // The I2C address of IMU2.
        int imu3_address;                             // The I2C address of IMU3.
        std::string magn_bus;                         // The I2C bus on which to find the magnetometer.
        int magn_address;                             // The I2C address of the magnetometer.
        std::string pressure_bus;                     // The I2C bus of the pressure sensor.
        int pressure_address;                         // The I2C address of the pressure sensor.
        int node_id;                                  // A unique ID number for the node.
        double altitude_filter_weight;
        double pressure_offset;
        int correct_with_yaw_diff = 0;
        int use_uwb_positioning = 0;
        double altimeter_z_accel_stdev;
        double pressure_altitude_stdev;
        int use_rs2 = 0;
        int uubla_baud_rate;
        std::string uubla_serial_port;

        /** \brief Overwrite the content of the configuration file associated with this struct.
         * \param option The configuration option to overwrite.
         * \param new_value The new new value of the parameter to be put in the file.
         */
        template <typename T>
        void replace(const std::string &option, const T new_value)
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
        template <typename T>
        arwain::ReturnCode read_option(const std::map<std::string, std::string> &options, const std::string &option, T &storage)
        {
            try
            {
                std::stringstream(options.at(option)) >> storage;
            }
            catch (const std::exception& e)
            {
                std::cout << "Failed to read option from configuration file\n";
                throw std::invalid_argument{option};
            }
            return arwain::ReturnCode::Success;
        }
    };
} // end namespace arwain

#endif
