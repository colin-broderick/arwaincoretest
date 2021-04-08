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
arwain::Configuration arwain::get_configuration(const std::string &filename)
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
        std::string name = line.substr(0, delimiter);
        std::string value = line.substr(delimiter + 1);
        options[name] = value;
    }

    // TODO Detect attempted read of non-existing options.

    // Read all options into a configuration object.
    arwain::Configuration cf;
    stringstream(options["active_threshold"]) >> cf.active_threshold;
    stringstream(options["walking_threshold"]) >> cf.walking_threshold;
    stringstream(options["running_threshold"]) >> cf.running_threshold;
    stringstream(options["crawling_threshold"]) >> cf.crawling_threshold;
    stringstream(options["climbing_threshold"]) >> cf.climbing_threshold;
    stringstream(options["gravity"]) >> cf.gravity;
    stringstream(options["struggle_threshold"]) >> cf.struggle_threshold;
    stringstream(options["freefall_sensitivity"]) >> cf.freefall_sensitivity;
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
    stringstream(options["use_indoor_positioning_system"]) >> cf.use_indoor_positioning_system;
    stringstream(options["orientation_filter"]) >> cf.orientation_filter;
    stringstream(options["inference_model_xml"]) >> cf.inference_model_xml;
    stringstream(options["use_pressure"]) >> cf.use_pressure;
    stringstream(options["sea_level_pressure"]) >> cf.sea_level_pressure;
    
    // Apply LoRa settings
    stringstream(options["lora_tx_power"]) >> cf.lora_tx_power;
    stringstream(options["lora_packet_frequency"]) >> cf.lora_packet_frequency;

    // Apply LoRa radio frequency setting with default 868 MHz.
    std::string rf = options["lora_rf_frequency"];
    if (rf == "433")
        cf.lora_rf_frequency = LoRa::FREQ_433;
    else if (rf == "868")
        cf.lora_rf_frequency = LoRa::FREQ_868;
    else if (rf == "915")
        cf.lora_rf_frequency = LoRa::FREQ_915;
    else
        cf.lora_rf_frequency = LoRa::FREQ_868;

    // Apply LoRa spread factor setting with default 12.
    std::string spreadfactor = options["lora_spread_factor"];
    if (spreadfactor == "6")
        cf.lora_spread_factor = LoRa::SF_6;
    else if (spreadfactor == "7")
        cf.lora_spread_factor = LoRa::SF_7;
    else if (spreadfactor == "8")
        cf.lora_spread_factor = LoRa::SF_8;
    else if (spreadfactor == "9")
        cf.lora_spread_factor = LoRa::SF_9;
    else if (spreadfactor == "10")
        cf.lora_spread_factor = LoRa::SF_10;
    else if (spreadfactor == "11")
        cf.lora_spread_factor = LoRa::SF_11;
    else if (spreadfactor == "12")
        cf.lora_spread_factor = LoRa::SF_12;
    else
        cf.lora_spread_factor = LoRa::SF_12;    

    // Apply LoRa bandwidth setting with default 125k.
    std::string bandwidth = options["lora_bandwidth"];
    if (bandwidth == "7.8")
        cf.lora_bandwidth = LoRa::BW_7k8;
    else if (bandwidth == "10.4")
        cf.lora_bandwidth = LoRa::BW_10k4;
    else if (bandwidth == "15.6")
        cf.lora_bandwidth = LoRa::BW_15k6;
    else if (bandwidth == "20.8")
        cf.lora_bandwidth = LoRa::BW_20k8;
    else if (bandwidth == "31.25")
        cf.lora_bandwidth = LoRa::BW_31k25;
    else if (bandwidth == "41.7")
        cf.lora_bandwidth = LoRa::BW_41k7;
    else if (bandwidth == "62.5")
        cf.lora_bandwidth = LoRa::BW_62k5;
    else if (bandwidth == "125")
        cf.lora_bandwidth = LoRa::BW_125k;
    else if (bandwidth == "250")
        cf.lora_bandwidth = LoRa::BW_250k;
    else if (bandwidth == "500")
        cf.lora_bandwidth = LoRa::BW_500k;
    else
        cf.lora_bandwidth = LoRa::BW_125k;

    // Apply LoRa coding rate with default 48.
    std::string codingrate = options["lora_coding_rate"];
    if (codingrate == "45")
        cf.lora_coding_rate = LoRa::CR_45;
    else if (codingrate == "46")
        cf.lora_coding_rate = LoRa::CR_46;
    else if (codingrate == "47")
        cf.lora_coding_rate = LoRa::CR_47;
    else if (codingrate == "48")
        cf.lora_coding_rate = LoRa::CR_48;
    else
        cf.lora_coding_rate = LoRa::CR_48;

    // Apply LoRa header mode with implicit as default.
    if (options["lora_header_mode"] == "explicit")
        cf.lora_header_mode = LoRa::HM_EXPLICIT;
    else
        cf.lora_header_mode = LoRa::HM_IMPLICIT;

    stringstream(options["lora_sync_word"]) >> cf.lora_sync_word;
    stringstream(options["lora_enable_crc"]) >> cf.lora_enable_crc;

    return cf;
}

std::ostream& operator<<(std::ostream& stream, const std::array<double, 3>& vector)
{
    stream << "vector3(" << vector[0] << ", " << vector[1] << ", " << vector[2] << ")";
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