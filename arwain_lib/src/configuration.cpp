#include "arwain/configuration.hpp"
#include "arwain/arwain.hpp"

/** \brief Create a configuration object based on supplied filename.
 * \param filename The location of the configuration file to read.
 * \return int return code.
 */
arwain::ReturnCode arwain::Configuration::read_from_file()
{
    // Open the configuration file name.
    std::ifstream file(this->config_file);
    if (!file.is_open())
    {
        return arwain::ReturnCode::NoConfigurationFile;
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
    read_option(options, "accel1_scale_x", this->accel1_scale.x);
    read_option(options, "accel1_scale_y", this->accel1_scale.y);
    read_option(options, "accel1_scale_z", this->accel1_scale.z);
    read_option(options, "accel2_scale_x", this->accel2_scale.x);
    read_option(options, "accel2_scale_y", this->accel2_scale.y);
    read_option(options, "accel2_scale_z", this->accel2_scale.z);
    read_option(options, "accel3_scale_x", this->accel3_scale.x);
    read_option(options, "accel3_scale_y", this->accel3_scale.y);
    read_option(options, "accel3_scale_z", this->accel3_scale.z);
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
    read_option(options, "mag_scale_xy", this->mag_scale_xy);
    read_option(options, "mag_scale_xz", this->mag_scale_yz);
    read_option(options, "mag_scale_yz", this->mag_scale_xz);
    read_option(options, "use_magnetometer", this->use_magnetometer);
    read_option(options, "log_magnetometer", this->log_magnetometer);
    read_option(options, "use_ips", this->use_ips);
    read_option(options, "npu_vel_weight_confidence", this->npu_vel_weight_confidence);
    read_option(options, "madgwick_beta", this->madgwick_beta);
    read_option(options, "madgwick_beta_conv", this->madgwick_beta_conv);
    read_option(options, "orientation_filter", this->orientation_filter);
    read_option(options, "altimeter_z_accel_stdev", this->altimeter_z_accel_stdev);
    read_option(options, "pressure_altitude_stdev", this->pressure_altitude_stdev);
    read_option(options, "use_rs2", this->use_rs2);
    read_option(options, "uubla_baud_rate", this->uubla_baud_rate);
    read_option(options, "uubla_serial_port", this->uubla_serial_port);

    // We want to fail out if the model XML file cannot be found.
    read_option(options, "inference_model_xml", this->inference_model_xml);
    if (!std::filesystem::exists(this->inference_model_xml))
    {
        return arwain::ReturnCode::NoInferenceXML;
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
    read_option(options, "altitude_filter_weight", this->altitude_filter_weight);
    read_option(options, "pressure_offset", this->pressure_offset);
    read_option(options, "correct_with_yaw_diff", this->correct_with_yaw_diff);

    // UWB options.
    read_option(options, "use_uwb_positioning", this->use_uwb_positioning);

    // Apply LoRa settings
    std::stringstream(options["lora_tx_power"]) >> this->lora_tx_power;
    std::stringstream(options["lora_packet_frequency"]) >> this->lora_packet_frequency;

    // Apply LoRa radio frequency setting with default 868 MHz.
    std::string rf = options["lora_rf_frequency"];
    if (rf == "433")
        this->lora_rf_frequency = LoRa<SPIDEVICEDRIVER>::Frequency::FREQ_433;
    else if (rf == "868")
        this->lora_rf_frequency = LoRa<SPIDEVICEDRIVER>::Frequency::FREQ_868;
    else if (rf == "915")
        this->lora_rf_frequency = LoRa<SPIDEVICEDRIVER>::Frequency::FREQ_915;
    else
        this->lora_rf_frequency = LoRa<SPIDEVICEDRIVER>::Frequency::FREQ_868;

    // Apply LoRa spread factor setting with default 12.
    std::string spreadfactor = options["lora_spread_factor"];
    if (spreadfactor == "6")
        this->lora_spread_factor = LoRa<SPIDEVICEDRIVER>::SpreadFactor::SF_6;
    else if (spreadfactor == "7")
        this->lora_spread_factor = LoRa<SPIDEVICEDRIVER>::SpreadFactor::SF_7;
    else if (spreadfactor == "8")
        this->lora_spread_factor = LoRa<SPIDEVICEDRIVER>::SpreadFactor::SF_8;
    else if (spreadfactor == "9")
        this->lora_spread_factor = LoRa<SPIDEVICEDRIVER>::SpreadFactor::SF_9;
    else if (spreadfactor == "10")
        this->lora_spread_factor = LoRa<SPIDEVICEDRIVER>::SpreadFactor::SF_10;
    else if (spreadfactor == "11")
        this->lora_spread_factor = LoRa<SPIDEVICEDRIVER>::SpreadFactor::SF_11;
    else if (spreadfactor == "12")
        this->lora_spread_factor = LoRa<SPIDEVICEDRIVER>::SpreadFactor::SF_12;
    else
        this->lora_spread_factor = LoRa<SPIDEVICEDRIVER>::SpreadFactor::SF_12;    

    // Apply LoRa bandwidth setting with default 125k.
    std::string bandwidth = options["lora_bandwidth"];
    if (bandwidth == "7.8")
        this->lora_bandwidth = LoRa<SPIDEVICEDRIVER>::Bandwidth::BW_7_8K;
    else if (bandwidth == "10.4")
        this->lora_bandwidth = LoRa<SPIDEVICEDRIVER>::Bandwidth::BW_10_4K;
    else if (bandwidth == "15.6")
        this->lora_bandwidth = LoRa<SPIDEVICEDRIVER>::Bandwidth::BW_15_6K;
    else if (bandwidth == "20.8")
        this->lora_bandwidth = LoRa<SPIDEVICEDRIVER>::Bandwidth::BW_20_8K;
    else if (bandwidth == "31.25")
        this->lora_bandwidth = LoRa<SPIDEVICEDRIVER>::Bandwidth::BW_31_25K;
    else if (bandwidth == "41.7")
        this->lora_bandwidth = LoRa<SPIDEVICEDRIVER>::Bandwidth::BW_41_7K;
    else if (bandwidth == "62.5")
        this->lora_bandwidth = LoRa<SPIDEVICEDRIVER>::Bandwidth::BW_62_5K;
    else if (bandwidth == "125")
        this->lora_bandwidth = LoRa<SPIDEVICEDRIVER>::Bandwidth::BW_125K;
    else if (bandwidth == "250")
        this->lora_bandwidth = LoRa<SPIDEVICEDRIVER>::Bandwidth::BW_250K;
    else if (bandwidth == "500")
        this->lora_bandwidth = LoRa<SPIDEVICEDRIVER>::Bandwidth::BW_500K;
    else
        this->lora_bandwidth = LoRa<SPIDEVICEDRIVER>::Bandwidth::BW_500K;

    // Apply LoRa coding rate with default 48.
    std::string codingrate = options["lora_coding_rate"];
    if (codingrate == "45")
        this->lora_coding_rate = LoRa<SPIDEVICEDRIVER>::CodingRate::CR_45;
    else if (codingrate == "46")
        this->lora_coding_rate = LoRa<SPIDEVICEDRIVER>::CodingRate::CR_46;
    else if (codingrate == "47")
        this->lora_coding_rate = LoRa<SPIDEVICEDRIVER>::CodingRate::CR_47;
    else if (codingrate == "48")
        this->lora_coding_rate = LoRa<SPIDEVICEDRIVER>::CodingRate::CR_48;
    else
        this->lora_coding_rate = LoRa<SPIDEVICEDRIVER>::CodingRate::CR_48;

    // Apply LoRa header mode with implicit as default.
    if (options["lora_header_mode"] == "explicit")
        this->lora_header_mode = LoRa<SPIDEVICEDRIVER>::HeaderMode::HM_EXPLICIT;
    else
        this->lora_header_mode = LoRa<SPIDEVICEDRIVER>::HeaderMode::HM_IMPLICIT;

    std::stringstream(options["lora_sync_word"]) >> this->lora_sync_word;
    std::stringstream(options["lora_enable_crc"]) >> this->lora_enable_crc;

    if (this->log_to_stdout)
    {
        std::cout << "Configuration file read successfully\n";
    }

    return arwain::ReturnCode::Success;
}

arwain::Configuration::Configuration(const arwain::InputParser& input)
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

    // If alternate configuration file supplied, read it instead of default.
    if (input.contains("-conf"))
    {
        this->config_file = input.get_cmd_option("-conf");
    }
}
