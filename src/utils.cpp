#include <csignal>
#include <sstream>
#include <thread>
#include <ctime>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <array>

#include "utils.h"
#include "imu_utils.h"

float arwain::getCPUTemp()
{
    float ret;
    std::array<char, 32> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen("vcgencmd measure_temp", "r"), pclose);
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    std::stringstream{result.substr(5, 4)} >> ret;
    return ret;
}

/** \brief Get the current system datetime as a string.
 * \return Datetime as string.
 */
std::string arwain::datetimestring()
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

/** \brief Utility functional for checking that the IMU is operational.
 */
void arwain::test_imu(int &shutdown)
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
    std::chrono::milliseconds interval{10};

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

/** \brief Create a configuration object based on supplied filename.
 * \param filename The location of the configuration file to read.
 * \return A Configuration object containing parameters read from the file.
 */
int arwain::get_configuration(const std::string &filename, arwain::Configuration& output_config)
{
    // TODO Checking file existence using std::filesystem would be preferred here but I had some build issue.
    // Open the configuration file name.
    std::ifstream file(filename);
    if (!file.is_open())
    {
        return 0;
    }

    // A map to store key value pairs from the configuration file.
    std::map<std::string, std::string> options;

    // Read each line into the map, based on the format "key=value".
    std::string line;
    while (std::getline(file, line))
    {
        if (line[0] == '[' || line.empty())
        {
            continue;
        }
        auto delimiter = line.find("=");
        std::string name = line.substr(0, delimiter);
        std::string value = line.substr(delimiter + 1);
        options[name] = value;
    }

    // TODO Detect attempted read of non-existing options.

    // Read all options into a configuration object.
    std::stringstream(options["active_threshold"]) >> output_config.active_threshold;
    std::stringstream(options["walking_threshold"]) >> output_config.walking_threshold;
    std::stringstream(options["running_threshold"]) >> output_config.running_threshold;
    std::stringstream(options["crawling_threshold"]) >> output_config.crawling_threshold;
    std::stringstream(options["climbing_threshold"]) >> output_config.climbing_threshold;
    std::stringstream(options["gravity"]) >> output_config.gravity;
    std::stringstream(options["struggle_threshold"]) >> output_config.struggle_threshold;
    std::stringstream(options["freefall_sensitivity"]) >> output_config.freefall_sensitivity;
    std::stringstream(options["accel_bias_x"]) >> output_config.accel_bias.x;
    std::stringstream(options["accel_bias_y"]) >> output_config.accel_bias.y;
    std::stringstream(options["accel_bias_z"]) >> output_config.accel_bias.z;
    std::stringstream(options["gyro_bias_x"]) >> output_config.gyro_bias.x;
    std::stringstream(options["gyro_bias_y"]) >> output_config.gyro_bias.y;
    std::stringstream(options["gyro_bias_z"]) >> output_config.gyro_bias.z;
    std::stringstream(options["mag_bias_x"]) >> output_config.mag_bias.x;
    std::stringstream(options["mag_bias_y"]) >> output_config.mag_bias.y;
    std::stringstream(options["mag_bias_z"]) >> output_config.mag_bias.z;
    std::stringstream(options["mag_scale_x"]) >> output_config.mag_scale.x;
    std::stringstream(options["mag_scale_y"]) >> output_config.mag_scale.y;
    std::stringstream(options["mag_scale_z"]) >> output_config.mag_scale.z;
    std::stringstream(options["use_magnetometer"]) >> output_config.use_magnetometer;
    std::stringstream(options["log_magnetometer"]) >> output_config.log_magnetometer;
    std::stringstream(options["npu_vel_weight_confidence"]) >> output_config.npu_vel_weight_confidence;
    std::stringstream(options["madgwick_beta"]) >> output_config.madgwick_beta;
    std::stringstream(options["use_indoor_positioning_system"]) >> output_config.use_indoor_positioning_system;
    std::stringstream(options["orientation_filter"]) >> output_config.orientation_filter;
    std::stringstream(options["inference_model_xml"]) >> output_config.inference_model_xml;
    std::stringstream(options["use_pressure"]) >> output_config.use_pressure;
    std::stringstream(options["sea_level_pressure"]) >> output_config.sea_level_pressure;
    
    // Apply LoRa settings
    std::stringstream(options["lora_tx_power"]) >> output_config.lora_tx_power;
    std::stringstream(options["lora_packet_frequency"]) >> output_config.lora_packet_frequency;

    // Apply LoRa radio frequency setting with default 868 MHz.
    std::string rf = options["lora_rf_frequency"];
    if (rf == "433")
        output_config.lora_rf_frequency = LoRa::FREQ_433;
    else if (rf == "868")
        output_config.lora_rf_frequency = LoRa::FREQ_868;
    else if (rf == "915")
        output_config.lora_rf_frequency = LoRa::FREQ_915;
    else
        output_config.lora_rf_frequency = LoRa::FREQ_868;

    // Apply LoRa spread factor setting with default 12.
    std::string spreadfactor = options["lora_spread_factor"];
    if (spreadfactor == "6")
        output_config.lora_spread_factor = LoRa::SF_6;
    else if (spreadfactor == "7")
        output_config.lora_spread_factor = LoRa::SF_7;
    else if (spreadfactor == "8")
        output_config.lora_spread_factor = LoRa::SF_8;
    else if (spreadfactor == "9")
        output_config.lora_spread_factor = LoRa::SF_9;
    else if (spreadfactor == "10")
        output_config.lora_spread_factor = LoRa::SF_10;
    else if (spreadfactor == "11")
        output_config.lora_spread_factor = LoRa::SF_11;
    else if (spreadfactor == "12")
        output_config.lora_spread_factor = LoRa::SF_12;
    else
        output_config.lora_spread_factor = LoRa::SF_12;    

    // Apply LoRa bandwidth setting with default 125k.
    std::string bandwidth = options["lora_bandwidth"];
    if (bandwidth == "7.8")
        output_config.lora_bandwidth = LoRa::BW_7k8;
    else if (bandwidth == "10.4")
        output_config.lora_bandwidth = LoRa::BW_10k4;
    else if (bandwidth == "15.6")
        output_config.lora_bandwidth = LoRa::BW_15k6;
    else if (bandwidth == "20.8")
        output_config.lora_bandwidth = LoRa::BW_20k8;
    else if (bandwidth == "31.25")
        output_config.lora_bandwidth = LoRa::BW_31k25;
    else if (bandwidth == "41.7")
        output_config.lora_bandwidth = LoRa::BW_41k7;
    else if (bandwidth == "62.5")
        output_config.lora_bandwidth = LoRa::BW_62k5;
    else if (bandwidth == "125")
        output_config.lora_bandwidth = LoRa::BW_125k;
    else if (bandwidth == "250")
        output_config.lora_bandwidth = LoRa::BW_250k;
    else if (bandwidth == "500")
        output_config.lora_bandwidth = LoRa::BW_500k;
    else
        output_config.lora_bandwidth = LoRa::BW_125k;

    // Apply LoRa coding rate with default 48.
    std::string codingrate = options["lora_coding_rate"];
    if (codingrate == "45")
        output_config.lora_coding_rate = LoRa::CR_45;
    else if (codingrate == "46")
        output_config.lora_coding_rate = LoRa::CR_46;
    else if (codingrate == "47")
        output_config.lora_coding_rate = LoRa::CR_47;
    else if (codingrate == "48")
        output_config.lora_coding_rate = LoRa::CR_48;
    else
        output_config.lora_coding_rate = LoRa::CR_48;

    // Apply LoRa header mode with implicit as default.
    if (options["lora_header_mode"] == "explicit")
        output_config.lora_header_mode = LoRa::HM_EXPLICIT;
    else
        output_config.lora_header_mode = LoRa::HM_IMPLICIT;

    std::stringstream(options["lora_sync_word"]) >> output_config.lora_sync_word;
    std::stringstream(options["lora_enable_crc"]) >> output_config.lora_enable_crc;

    return 1;
}

std::ostream& operator<<(std::ostream& stream, const std::array<double, 3>& vector)
{
    stream << "vector3(" << vector[0] << ", " << vector[1] << ", " << vector[2] << ")";
    return stream;
}

/** \brief Print out a 6-wide IMU reading. */
std::ostream& operator<<(std::ostream& stream, const std::array<double, 6>& line)
{
    stream << "IMU_data(" << line[0] << ", " <<line[1] << ", " <<line[2] << ", " <<line[3] << ", " <<line[4] << ", " <<line[5] << ")";
    return stream;
}

std::array<double, 3> normalised(const std::array<double, 3>& vector)
{
    double invNorm = 1.0/sqrt(
        vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]
    );
    return std::array<double, 3>{
        vector[0] * invNorm,
        vector[1] * invNorm,
        vector[2] * invNorm
    };
}