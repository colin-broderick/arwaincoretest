#include <iostream>
#include <filesystem>
#include <iomanip>
#include <thread>
#include <cmath>

#include "arwain.hpp"
#include "input_parser.hpp"
#include "imu_reader.hpp"
#include "velocity_prediction.hpp"
#include "transmit_lora.hpp"
#include "std_output.hpp"
#include "indoor_positioning_wrapper.hpp"
#include "altimeter.hpp"
#include "logger.hpp"
#include "IMU_IIM42652_driver.hpp"
#include "madgwick.hpp"
#include "efaroe.hpp"
#include "lis3mdl.hpp"
#include "geomagnetic_orientation.hpp"

static double pi = 3.14159265;

// General configuration data.
namespace arwain
{
    int shutdown = 0;
    arwain::Configuration config;
    std::string folder_date_string;
    arwain::Status status;
    arwain::Logger error_log;
}

// Shared data buffers; mutex locks must be used when accessing.
namespace arwain::Buffers
{
    std::deque<vector6> IMU_BUFFER{arwain::BufferSizes::IMU_BUFFER_LEN};
    std::deque<vector6> IMU_WORLD_BUFFER{arwain::BufferSizes::IMU_BUFFER_LEN};
    std::deque<vector3> VELOCITY_BUFFER{arwain::BufferSizes::VELOCITY_BUFFER_LEN};
    std::deque<vector3> POSITION_BUFFER{arwain::BufferSizes::POSITION_BUFFER_LEN};
    std::deque<vector3> MAG_BUFFER{arwain::BufferSizes::MAG_BUFFER_LEN};
    std::deque<vector3> MAG_WORLD_BUFFER{arwain::BufferSizes::MAG_BUFFER_LEN};
    std::deque<vector3> IPS_BUFFER{arwain::BufferSizes::IPS_BUFFER_LEN};
    std::deque<vector3> PRESSURE_BUFFER{arwain::BufferSizes::PRESSURE_BUFFER_LEN};
    std::deque<euler_orientation_t> EULER_ORIENTATION_BUFFER{arwain::BufferSizes::ORIENTATION_BUFFER_LEN};
    std::deque<quaternion> QUAT_ORIENTATION_BUFFER{arwain::BufferSizes::ORIENTATION_BUFFER_LEN};
    std::deque<quaternion> MAG_ORIENTATION_BUFFER{arwain::BufferSizes::MAG_ORIENTATION_BUFFER_LEN};
}

// Mutex locks for use when accessing shared buffers.
namespace arwain::Locks
{
    std::mutex IMU_BUFFER_LOCK;
    std::mutex MAG_BUFFER_LOCK;
    std::mutex VELOCITY_BUFFER_LOCK;
    std::mutex STATUS_FLAG_LOCK;
    std::mutex POSITION_BUFFER_LOCK;
    std::mutex ORIENTATION_BUFFER_LOCK;
    std::mutex PRESSURE_BUFFER_LOCK;
}

/** \brief Start a Python script which opens a socket and does inference on data we send it. */
static void py_inference()
{
    if (!arwain::config.no_inference)
    {   
        std::string command = "python3 ./python_utils/ncs2_interface.py " + arwain::config.inference_model_xml + " > /dev/null &";
        system(command.c_str());
    }
}

/** \brief Utility functional for checking that the IMU is operational.
 */
int arwain::test_imu()
{
    // Initialize the IMU.
    IMU_IIM42652 imu{0x68, "/dev/i2c-1"};
    // BMI270 imu{0x69, "/dev/i2c-1"};
    // Multi_IIM42652 imu;

    // Local buffers for IMU data
    vector3 accel_data;
    vector3 gyro_data;

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{10};

    while (!arwain::shutdown)
    {
        // bmi270_h.get_bmi270_data(&accel_data, &gyro_data);
        imu.read_IMU();
        accel_data = {
            imu.accelerometer_x,
            imu.accelerometer_y,
            imu.accelerometer_z
        };
        gyro_data = {
            imu.gyroscope_x,
            imu.gyroscope_y,
            imu.gyroscope_z,
        };

        // Display IMU data.
        std::cout << time.time_since_epoch().count() << std::fixed << std::right << std::setprecision(3) << "\t" << accel_data.x << "\t" << accel_data.y << "\t" << accel_data.z << "\t" << gyro_data.x << "\t" << gyro_data.y << "\t" << gyro_data.z << "\n";

        // Wait until the next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);

    }

    return arwain::ExitCodes::Success;
}

/** \brief Checks that the correct chip ID can be read from the magnetometer. If so, reads and prints orientation until interrupted. */
int arwain::test_mag()
{
    LIS3MDL magn{arwain::config.magn_address, arwain::config.magn_bus};
    magn.set_calibration(arwain::config.mag_bias, arwain::config.mag_scale);

    int id = magn.test_chip();
    if (id != 0x3D)
    {
        std::cout << "Chip ID incorrect: should be 0x3D, got " <<std::hex << std::showbase << id << std::endl;
        return arwain::ExitCodes::FailedMagnetometer;
    }
    std::cout << "Chip ID: " << std::hex << std::showbase << magn.test_chip() << std::dec << std::endl;

    while (!arwain::shutdown)
    {
        vector3 reading = magn.read();
        std::cout << std::atan2(reading.z, reading.y) * 180.0 / pi << " degrees from North" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }

    return arwain::ExitCodes::Success;
}

int arwain::test_lora_tx()
{
    LoRa lora{arwain::config.lora_address, false};

    if (lora.test_chip() == 0x1A)
    {
        std::cout << "Found chip" << std::endl;
    };

    std::string message = "ARWAIN.LoRa";
    std::cout << "Transmitting message \"" << message << ".x\" at 1 Hz ..." << std::endl;

    int i = 0;
    while (!arwain::shutdown)
    {
        i++;
        std::string msg = message + "." + std::to_string(i);
        lora.send_message(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds{1000});
    }

    return arwain::ExitCodes::Success;
}

int arwain::test_lora_rx()
{
    LoRa lora{arwain::config.lora_address, true};

    if (lora.test_chip() == 0x1A)
    {
        std::cout << "Found chip" << std::endl;
    };

    std::cout << "Receiving LoRa messages ..." << std::endl;

    while (!arwain::shutdown)
    {
        auto [rx, message] = lora.receive_string();
        if (rx)
        {
            std::cout << message << std::endl;
        }
    }

    return arwain::ExitCodes::Success;
}

void arwain::setup(const InputParser& input)
{
    // Create output directory and write copy of current configuration.
    if (arwain::config.log_to_file)
    {
        if (input.contains("-name"))
        {
            arwain::folder_date_string = "./data_" + input.getCmdOption("-name") + "_" + arwain::datetimestring();
        }
        else
        {
            arwain::folder_date_string = "./data_" + arwain::datetimestring();
        }
        if (!std::filesystem::is_directory(arwain::folder_date_string))
        {
            std::filesystem::create_directory(arwain::folder_date_string);
        }
        std::filesystem::copy(arwain::config.config_file, arwain::folder_date_string + "/config.conf");
    }

    // Open error log file (globally accessible)
    if (arwain::config.log_to_file)
    {
        arwain::error_log.open(arwain::folder_date_string + "/ERRORS.txt");
        arwain::error_log << "time event" << "\n";
    }
}

/** \brief Create a configuration object based on supplied filename.
 * \param filename The location of the configuration file to read.
 * \return A Configuration object containing parameters read from the file.
 */
int arwain::Configuration::read_from_file()
{
    // Open the configuration file name.
    std::ifstream file(this->config_file);
    if (!file.is_open())
    {
        return arwain::ExitCodes::FailedConfiguration;
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

    // Read all options into the configuration object.
    read_option(options, "active_threshold", this->active_threshold);
    read_option(options, "walking_threshold", this->walking_threshold);
    read_option(options, "running_threshold", this->running_threshold);
    read_option(options, "crawling_threshold", this->crawling_threshold);
    read_option(options, "climbing_threshold", this->climbing_threshold);
    read_option(options, "gravity", this->gravity);
    read_option(options, "struggle_threshold", this->struggle_threshold);
    read_option(options, "freefall_sensitivity", this->freefall_sensitivity);
    read_option(options, "accel1_bias_x", this->accel1_bias.x);
    read_option(options, "accel1_bias_y", this->accel1_bias.y);
    read_option(options, "accel1_bias_z", this->accel1_bias.z);
    read_option(options, "accel2_bias_x", this->accel2_bias.x);
    read_option(options, "accel2_bias_y", this->accel2_bias.y);
    read_option(options, "accel2_bias_z", this->accel2_bias.z);
    read_option(options, "accel3_bias_x", this->accel3_bias.x);
    read_option(options, "accel3_bias_y", this->accel3_bias.y);
    read_option(options, "accel3_bias_z", this->accel3_bias.z);
    read_option(options, "gyro1_bias_x", this->gyro1_bias.x);
    read_option(options, "gyro1_bias_y", this->gyro1_bias.y);
    read_option(options, "gyro1_bias_z", this->gyro1_bias.z);
    read_option(options, "gyro2_bias_x", this->gyro2_bias.x);
    read_option(options, "gyro2_bias_y", this->gyro2_bias.y);
    read_option(options, "gyro2_bias_z", this->gyro2_bias.z);
    read_option(options, "gyro3_bias_x", this->gyro3_bias.x);
    read_option(options, "gyro3_bias_y", this->gyro3_bias.y);
    read_option(options, "gyro3_bias_z", this->gyro3_bias.z);
    read_option(options, "mag_bias_x", this->mag_bias.x);
    read_option(options, "mag_bias_y", this->mag_bias.y);
    read_option(options, "mag_bias_z", this->mag_bias.z);
    read_option(options, "mag_scale_x", this->mag_scale.x);
    read_option(options, "mag_scale_y", this->mag_scale.y);
    read_option(options, "mag_scale_z", this->mag_scale.z);
    read_option(options, "use_magnetometer", this->use_magnetometer);
    read_option(options, "log_magnetometer", this->log_magnetometer);
    read_option(options, "npu_vel_weight_confidence", this->npu_vel_weight_confidence);
    read_option(options, "madgwick_beta", this->madgwick_beta);
    read_option(options, "use_indoor_positioning_system", this->use_indoor_positioning_system);
    read_option(options, "orientation_filter", this->orientation_filter);

    // We want to fail out if the model XML file cannot be found.
    read_option(options, "inference_model_xml", this->inference_model_xml);
    if (!std::filesystem::exists(this->inference_model_xml))
    {
        return arwain::ExitCodes::InferenceXMLMissing;
    }

    read_option(options, "sea_level_pressure", this->sea_level_pressure);
    read_option(options, "imu1_bus", this->imu1_bus);
    read_option(options, "imu2_bus", this->imu2_bus);
    read_option(options, "imu3_bus", this->imu3_bus);
    read_option(options, "imu1_address", this->imu1_address);
    read_option(options, "imu2_address", this->imu2_address);
    read_option(options, "imu3_address", this->imu3_address);
    read_option(options, "magn_address", this->magn_address);
    read_option(options, "magn_bus", this->magn_bus);
    read_option(options, "pressure_address", this->pressure_address);
    read_option(options, "pressure_bus", this->pressure_bus);
    read_option(options, "lora_address", this->lora_address);
    read_option(options, "node_id", this->node_id);
    
    // Apply LoRa settings
    std::stringstream(options["lora_tx_power"]) >> this->lora_tx_power;
    std::stringstream(options["lora_packet_frequency"]) >> this->lora_packet_frequency;

    // Apply LoRa radio frequency setting with default 868 MHz.
    std::string rf = options["lora_rf_frequency"];
    if (rf == "433")
        this->lora_rf_frequency = LoRa::Frequency::FREQ_433;
    else if (rf == "868")
        this->lora_rf_frequency = LoRa::Frequency::FREQ_868;
    else if (rf == "915")
        this->lora_rf_frequency = LoRa::Frequency::FREQ_915;
    else
        this->lora_rf_frequency = LoRa::Frequency::FREQ_868;

    // Apply LoRa spread factor setting with default 12.
    std::string spreadfactor = options["lora_spread_factor"];
    if (spreadfactor == "6")
        this->lora_spread_factor = LoRa::SpreadFactor::SF_6;
    else if (spreadfactor == "7")
        this->lora_spread_factor = LoRa::SpreadFactor::SF_7;
    else if (spreadfactor == "8")
        this->lora_spread_factor = LoRa::SpreadFactor::SF_8;
    else if (spreadfactor == "9")
        this->lora_spread_factor = LoRa::SpreadFactor::SF_9;
    else if (spreadfactor == "10")
        this->lora_spread_factor = LoRa::SpreadFactor::SF_10;
    else if (spreadfactor == "11")
        this->lora_spread_factor = LoRa::SpreadFactor::SF_11;
    else if (spreadfactor == "12")
        this->lora_spread_factor = LoRa::SpreadFactor::SF_12;
    else
        this->lora_spread_factor = LoRa::SpreadFactor::SF_12;    

    // Apply LoRa bandwidth setting with default 125k.
    std::string bandwidth = options["lora_bandwidth"];
    if (bandwidth == "7.8")
        this->lora_bandwidth = LoRa::Bandwidth::BW_7_8K;
    else if (bandwidth == "10.4")
        this->lora_bandwidth = LoRa::Bandwidth::BW_10_4K;
    else if (bandwidth == "15.6")
        this->lora_bandwidth = LoRa::Bandwidth::BW_15_6K;
    else if (bandwidth == "20.8")
        this->lora_bandwidth = LoRa::Bandwidth::BW_20_8K;
    else if (bandwidth == "31.25")
        this->lora_bandwidth = LoRa::Bandwidth::BW_31_25K;
    else if (bandwidth == "41.7")
        this->lora_bandwidth = LoRa::Bandwidth::BW_41_7K;
    else if (bandwidth == "62.5")
        this->lora_bandwidth = LoRa::Bandwidth::BW_62_5K;
    else if (bandwidth == "125")
        this->lora_bandwidth = LoRa::Bandwidth::BW_125K;
    else if (bandwidth == "250")
        this->lora_bandwidth = LoRa::Bandwidth::BW_250K;
    else if (bandwidth == "500")
        this->lora_bandwidth = LoRa::Bandwidth::BW_500K;
    else
        this->lora_bandwidth = LoRa::Bandwidth::BW_125K;

    // Apply LoRa coding rate with default 48.
    std::string codingrate = options["lora_coding_rate"];
    if (codingrate == "45")
        this->lora_coding_rate = LoRa::CodingRate::CR_45;
    else if (codingrate == "46")
        this->lora_coding_rate = LoRa::CodingRate::CR_46;
    else if (codingrate == "47")
        this->lora_coding_rate = LoRa::CodingRate::CR_47;
    else if (codingrate == "48")
        this->lora_coding_rate = LoRa::CodingRate::CR_48;
    else
        this->lora_coding_rate = LoRa::CodingRate::CR_48;

    // Apply LoRa header mode with implicit as default.
    if (options["lora_header_mode"] == "explicit")
        this->lora_header_mode = LoRa::HeaderMode::HM_EXPLICIT;
    else
        this->lora_header_mode = LoRa::HeaderMode::HM_IMPLICIT;

    std::stringstream(options["lora_sync_word"]) >> this->lora_sync_word;
    std::stringstream(options["lora_enable_crc"]) >> this->lora_enable_crc;

    if (this->log_to_stdout)
    {
        std::cout << "Configuration file read successfully\n";
    }

    return arwain::ExitCodes::Success;
}

arwain::Configuration::Configuration(const InputParser& input)
{    
    // Enable/disable stdout logging.
    if (input.contains("-lstd"))
    {
        this->log_to_stdout = 1;
    }

    // Disable/enable velocity inference.
    if (input.contains("-noinf"))
    {
        this->no_inference = 1;
    }

    // Disable/enable LoRa transmission.
    if (input.contains("-nolora"))
    {
        this->no_lora = 1;
    }

    if (input.contains("-nopressure"))
    {
        this->no_pressure = 1;
    }

    // Disable/enable IMU.
    if (input.contains("-noimu"))
    {
        this->no_imu = 1;
    }

    // Enable/disable logging to file.
    if (input.contains("-lfile"))
    {
        std::cout << "Logging to file" << "\n";
        this->log_to_file = 1;
    }

    // If alternate configuration file supplied, read it instead of default.
    if (input.contains("-conf"))
    {
        this->config_file = input.getCmdOption("-conf");
    }

    // If neither file nor std logging are enabled, warn the user that no data will be logged.
    if (!input.contains("-lstd") && !input.contains("-lfile"))
    {
        std::cerr << "No logging enabled - you probably want to use -lstd or -lfile or both" << "\n";
    }
}

int arwain::execute_inference()
{
    // Start worker threads.
    std::thread imu_reader_thread(imu_reader);                   // Reading IMU data, updating orientation filters.
    std::thread predict_velocity_thread(predict_velocity);       // Velocity and position inference.
    std::thread stance_detector_thread(stance_detector);         // Stance, freefall, entanglement detection.
    std::thread transmit_lora_thread(transmit_lora);             // LoRa packet transmissions.
    std::thread std_output_thread(std_output);                   // Prints useful output to std out.
    std::thread indoor_positioning_thread(indoor_positioning);   // Floor, stair, corner snapping.
    std::thread altimeter_thread(altimeter);                     // Uses the BMP280 sensor to determine altitude.
    std::thread py_inference_thread{py_inference};               // Temporary: Run Python script to handle velocity inference.
    // std::thread magnetometer_thread{mag_reader};                 // Reads the magnetic sensor and computes geomagnetic orientation.
    // std::thread py_transmitter_thread{py_transmitter};           // Temporary: Run Python script to handle LoRa transmission.
    // std::thread kalman_filter(kalman);                           // Experimental: Fuse IMU reading and pressure reading for altitude.

    // Wait for all threads to terminate.
    imu_reader_thread.join();
    predict_velocity_thread.join();
    stance_detector_thread.join();
    transmit_lora_thread.join();
    std_output_thread.join();
    indoor_positioning_thread.join();
    py_inference_thread.join();
    altimeter_thread.join();
    // magnetometer_thread.join();
    // py_transmitter_thread.join();
    // kalman_filter.join();

    return arwain::ExitCodes::Success;
}

float arwain::getPiCPUTemp()
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

static euler_orientation_t compute_euler(quaternion& q)
{
    euler_orientation_t euler;
    euler.roll = std::atan2(q.w*q.x + q.y*q.z, 0.5f - q.x*q.x - q.y*q.y)  * 180.0 / 3.14159;
	euler.pitch = std::asin(-2.0 * (q.x*q.z - q.w*q.y))  * 180.0 / 3.14159;
	euler.yaw = std::atan2(q.x*q.y + q.w*q.z, 0.5 - q.y*q.y - q.z*q.z)  * 180.0 / 3.14159;
    return euler;
}

int arwain::test_ori(int frequency)
{
    IMU_IIM42652 imu{config.imu1_address, config.imu1_bus};
    arwain::Madgwick filter{static_cast<double>(frequency), config.madgwick_beta};
    // arwain::eFaroe filter{{1, 0, 0, 0}, config.gyro1_bias, 0, config.efaroe_beta, config.efaroe_zeta};

    auto time = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds interval{1000/frequency};
    int count = 0;
    euler_orientation_t euler;
    quaternion quat;

    vector3 gyro;
    vector3 accel;
    std::cout << "Starting orientation filter at " << frequency << " Hz" << std::endl;

    while (!shutdown)
    {
        time += interval;
        auto timeCount = time.time_since_epoch().count();
        std::this_thread::sleep_until(time);

        imu.read_IMU();
        gyro = {imu.gyroscope_x, imu.gyroscope_y, imu.gyroscope_z};
        gyro = gyro - config.gyro1_bias;
        accel = {imu.accelerometer_x, imu.accelerometer_y, imu.accelerometer_z};
        accel = accel - config.accel1_bias;
        
        filter.update(timeCount, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);

        count++;
        if (count % frequency == 0)
        {
            quat = {filter.getW(), filter.getX(), filter.getY(), filter.getZ()};
            euler = compute_euler(quat);
            std::cout << "Quaternion: " << std::fixed << std::showpos << filter.getW() << " " << filter.getX() << " " << filter.getY() << " " << filter.getZ() << "\t\t";
            std::cout << "Euler: " << std::fixed << std::showpos << euler.roll << " " << euler.pitch << " " << euler.yaw << "\n";
        }
    }
    
    return arwain::ExitCodes::Success;
}

int arwain::calibrate_magnetometers()
{
    LIS3MDL magnetometer{arwain::config.magn_address, arwain::config.magn_bus};
    std::cout << "Move the device through all orientations for 10 seconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds{1000});
    std::cout << "Calibration log started" << std::endl;
    magnetometer.calibrate();
    std::cout << "Calibration log compelte" << std::endl;
    return arwain::ExitCodes::Success;
}

int arwain::calibrate_gyroscopes()
{
    vector3 results;

    IMU_IIM42652 imu1{arwain::config.imu1_address, arwain::config.imu1_bus};
    std::cout << "Calibrating gyroscope on " << arwain::config.imu1_bus << " at 0x" << std::hex << arwain::config.imu1_address << "; please wait" << std::endl;
    results = imu1.calibrate_gyroscope();
    std::cout << "Completed pass 1 on gyro 1" << std::endl;
    results = results + imu1.calibrate_gyroscope();
    std::cout << "Completed pass 2 on gyro 1" << std::endl;
    results = results + imu1.calibrate_gyroscope();
    std::cout << "Completed pass 3 on gyro 1" << std::endl;
    results = results / 3.0;
    arwain::config.replace("gyro1_bias_x", results.x);
    arwain::config.replace("gyro1_bias_y", results.y);
    arwain::config.replace("gyro1_bias_z", results.z);
    std::cout << "Calibration complete" << std::endl;

    IMU_IIM42652 imu2{arwain::config.imu2_address, arwain::config.imu2_bus};
    std::cout << "Calibrating gyroscope on " << arwain::config.imu2_bus << " at 0x" << std::hex << arwain::config.imu2_address << "; please wait" << std::endl;
    results = imu2.calibrate_gyroscope();
    std::cout << "Completed pass 1 on gyro 2" << std::endl;
    results = results + imu2.calibrate_gyroscope();
    std::cout << "Completed pass 2 on gyro 2" << std::endl;
    results = results + imu2.calibrate_gyroscope();
    std::cout << "Completed pass 3 on gyro 2" << std::endl;
    results = results / 3.0;
    std::cout << "Calibration complete" << std::endl;
    arwain::config.replace("gyro2_bias_x", results.x);
    arwain::config.replace("gyro2_bias_y", results.y);
    arwain::config.replace("gyro2_bias_z", results.z);

    IMU_IIM42652 imu3{arwain::config.imu3_address, arwain::config.imu3_bus};
    std::cout << "Calibrating gyroscope on " << arwain::config.imu3_bus << " at 0x" << std::hex << arwain::config.imu3_address << "; please wait" << std::endl;
    results = imu3.calibrate_gyroscope();
    std::cout << "Completed pass 1 on gyro 3" << std::endl;
    results = results + imu3.calibrate_gyroscope();
    std::cout << "Completed pass 2 on gyro 3" << std::endl;
    results = results + imu3.calibrate_gyroscope();
    std::cout << "Completed pass 3 on gyro 3" << std::endl;
    results = results / 3.0;
    std::cout << "Calibration complete" << std::endl;
    arwain::config.replace("gyro3_bias_x", results.x);
    arwain::config.replace("gyro3_bias_y", results.y);
    arwain::config.replace("gyro3_bias_z", results.z);

    std::cout << std::dec;

    return arwain::ExitCodes::Success;
}

int arwain::calibrate_accelerometers()
{
    vector3 results;

    IMU_IIM42652 imu1{arwain::config.imu1_address, arwain::config.imu1_bus};
    std::cout << "Calibrating accelerometer on " << arwain::config.imu1_bus << " at 0x" << std::hex << arwain::config.imu1_address << "; please wait" << std::endl;
    results = imu1.calibrate_accelerometer();
    std::cout << "Calibration complete" << std::endl;
    arwain::config.replace("accel1_bias_x", results.x);
    arwain::config.replace("accel1_bias_y", results.y);
    arwain::config.replace("accel1_bias_z", results.z);

    IMU_IIM42652 imu2{arwain::config.imu2_address, arwain::config.imu2_bus};
    std::cout << "Calibrating accelerometer on " << arwain::config.imu2_bus << " at 0x" << std::hex << arwain::config.imu2_address << "; please wait" << std::endl;
    results = imu2.calibrate_accelerometer();
    std::cout << "Calibration complete" << std::endl;
    arwain::config.replace("accel2_bias_x", results.x);
    arwain::config.replace("accel2_bias_y", results.y);
    arwain::config.replace("accel2_bias_z", results.z);

    IMU_IIM42652 imu3{arwain::config.imu3_address, arwain::config.imu3_bus};
    std::cout << "Calibrating accelerometer on " << arwain::config.imu3_bus << " at 0x" << std::hex << arwain::config.imu3_address << "; please wait" << std::endl;
    results = imu3.calibrate_accelerometer();
    std::cout << "Calibration complete" << std::endl;
    arwain::config.replace("accel3_bias_x", results.x);
    arwain::config.replace("accel3_bias_y", results.y);
    arwain::config.replace("accel3_bias_z", results.z);

    std::cout << std::dec;

    return arwain::ExitCodes::Success;
}
