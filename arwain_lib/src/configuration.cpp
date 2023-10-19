#include <arwain/config_parser.hpp>

#include "arwain/configuration.hpp"
#include "arwain/arwain.hpp"

/** \brief Create a configuration object based on supplied filename.
 * \param filename The location of the configuration file to read.
 * \return int return code.
 */
arwain::ReturnCode arwain::Configuration::read_from_file()
{
    ConfigParser cfgparser{this->config_file};

    // Read all options into the configuration object.
    cfgparser.read_option("active_threshold", this->active_threshold);
    cfgparser.read_option("walking_threshold", this->walking_threshold);
    cfgparser.read_option("running_threshold", this->running_threshold);
    cfgparser.read_option("crawling_threshold", this->crawling_threshold);
    cfgparser.read_option("climbing_threshold", this->climbing_threshold);
    cfgparser.read_option("gravity", this->gravity);
    cfgparser.read_option("struggle_threshold", this->struggle_threshold);
    cfgparser.read_option("freefall_sensitivity", this->freefall_sensitivity);
    cfgparser.read_option("accel1_bias_x", this->accel1_bias.x);
    cfgparser.read_option("accel1_bias_y", this->accel1_bias.y);
    cfgparser.read_option("accel1_bias_z", this->accel1_bias.z);
    cfgparser.read_option("accel2_bias_x", this->accel2_bias.x);
    cfgparser.read_option("accel2_bias_y", this->accel2_bias.y);
    cfgparser.read_option("accel2_bias_z", this->accel2_bias.z);
    cfgparser.read_option("accel3_bias_x", this->accel3_bias.x);
    cfgparser.read_option("accel3_bias_y", this->accel3_bias.y);
    cfgparser.read_option("accel3_bias_z", this->accel3_bias.z);
    cfgparser.read_option("accel1_scale_x", this->accel1_scale.x);
    cfgparser.read_option("accel1_scale_y", this->accel1_scale.y);
    cfgparser.read_option("accel1_scale_z", this->accel1_scale.z);
    cfgparser.read_option("accel2_scale_x", this->accel2_scale.x);
    cfgparser.read_option("accel2_scale_y", this->accel2_scale.y);
    cfgparser.read_option("accel2_scale_z", this->accel2_scale.z);
    cfgparser.read_option("accel3_scale_x", this->accel3_scale.x);
    cfgparser.read_option("accel3_scale_y", this->accel3_scale.y);
    cfgparser.read_option("accel3_scale_z", this->accel3_scale.z);
    cfgparser.read_option("gyro1_bias_x", this->gyro1_bias.x);
    cfgparser.read_option("gyro1_bias_y", this->gyro1_bias.y);
    cfgparser.read_option("gyro1_bias_z", this->gyro1_bias.z);
    cfgparser.read_option("gyro2_bias_x", this->gyro2_bias.x);
    cfgparser.read_option("gyro2_bias_y", this->gyro2_bias.y);
    cfgparser.read_option("gyro2_bias_z", this->gyro2_bias.z);
    cfgparser.read_option("gyro3_bias_x", this->gyro3_bias.x);
    cfgparser.read_option("gyro3_bias_y", this->gyro3_bias.y);
    cfgparser.read_option("gyro3_bias_z", this->gyro3_bias.z);
    cfgparser.read_option("mag_bias_x", this->mag_bias.x);
    cfgparser.read_option("mag_bias_y", this->mag_bias.y);
    cfgparser.read_option("mag_bias_z", this->mag_bias.z);
    cfgparser.read_option("mag_scale_x", this->mag_scale.x);
    cfgparser.read_option("mag_scale_y", this->mag_scale.y);
    cfgparser.read_option("mag_scale_z", this->mag_scale.z);
    cfgparser.read_option("mag_scale_xy", this->mag_scale_xy);
    cfgparser.read_option("mag_scale_xz", this->mag_scale_yz);
    cfgparser.read_option("mag_scale_yz", this->mag_scale_xz);
    cfgparser.read_option("use_magnetometer", this->use_magnetometer);
    cfgparser.read_option("log_magnetometer", this->log_magnetometer);
    cfgparser.read_option("use_ips", this->use_ips);
    cfgparser.read_option("npu_vel_weight_confidence", this->npu_vel_weight_confidence);
    cfgparser.read_option("madgwick_beta", this->madgwick_beta);
    cfgparser.read_option("madgwick_beta_conv", this->madgwick_beta_conv);
    cfgparser.read_option("orientation_filter", this->orientation_filter);
    cfgparser.read_option("altimeter_z_accel_stdev", this->altimeter_z_accel_stdev);
    cfgparser.read_option("pressure_altitude_stdev", this->pressure_altitude_stdev);
    cfgparser.read_option("use_rs2", this->use_rs2);
    cfgparser.read_option("uubla_baud_rate", this->uubla_baud_rate);
    cfgparser.read_option("uubla_serial_port", this->uubla_serial_port);

    // We want to fail out if the model XML file cannot be found.
    cfgparser.read_option("inference_model_xml", this->inference_model_xml);
    if (!std::filesystem::exists(this->inference_model_xml))
    {
        return arwain::ReturnCode::NoInferenceXML;
    }

    cfgparser.read_option("sea_level_pressure", this->sea_level_pressure);
    cfgparser.read_option("imu1_bus", this->imu1_bus);
    cfgparser.read_option("imu2_bus", this->imu2_bus);
    cfgparser.read_option("imu3_bus", this->imu3_bus);
    cfgparser.read_option("imu1_address", this->imu1_address);
    cfgparser.read_option("imu2_address", this->imu2_address);
    cfgparser.read_option("imu3_address", this->imu3_address);
    cfgparser.read_option("magn_address", this->magn_address);
    cfgparser.read_option("magn_bus", this->magn_bus);
    cfgparser.read_option("pressure_address", this->pressure_address);
    cfgparser.read_option("pressure_bus", this->pressure_bus);
    cfgparser.read_option("lora_address", this->lora_address);
    cfgparser.read_option("node_id", this->node_id);
    cfgparser.read_option("altitude_filter_weight", this->altitude_filter_weight);
    cfgparser.read_option("pressure_offset", this->pressure_offset);
    cfgparser.read_option("correct_with_yaw_diff", this->correct_with_yaw_diff);

    // UWB options.
    cfgparser.read_option("use_uwb_positioning", this->use_uwb_positioning);

    // Apply LoRa settings
    std::stringstream(cfgparser.options["lora_tx_power"]) >> this->lora_tx_power;
    std::stringstream(cfgparser.options["lora_packet_frequency"]) >> this->lora_packet_frequency;

    // Apply LoRa radio frequency setting with default 868 MHz.
    std::string rf = cfgparser.options["lora_rf_frequency"];
    if (rf == "433")
        this->lora_rf_frequency = RFM95W<LinuxSpiDevice>::Frequency::FREQ_433;
    else if (rf == "868")
        this->lora_rf_frequency = RFM95W<LinuxSpiDevice>::Frequency::FREQ_868;
    else if (rf == "915")
        this->lora_rf_frequency = RFM95W<LinuxSpiDevice>::Frequency::FREQ_915;
    else
        this->lora_rf_frequency = RFM95W<LinuxSpiDevice>::Frequency::FREQ_868;

    // Apply LoRa spread factor setting with default 12.
    std::string spreadfactor = cfgparser.options["lora_spread_factor"];
    if (spreadfactor == "6")
        this->lora_spread_factor = RFM95W<LinuxSpiDevice>::SpreadFactor::SF_6;
    else if (spreadfactor == "7")
        this->lora_spread_factor = RFM95W<LinuxSpiDevice>::SpreadFactor::SF_7;
    else if (spreadfactor == "8")
        this->lora_spread_factor = RFM95W<LinuxSpiDevice>::SpreadFactor::SF_8;
    else if (spreadfactor == "9")
        this->lora_spread_factor = RFM95W<LinuxSpiDevice>::SpreadFactor::SF_9;
    else if (spreadfactor == "10")
        this->lora_spread_factor = RFM95W<LinuxSpiDevice>::SpreadFactor::SF_10;
    else if (spreadfactor == "11")
        this->lora_spread_factor = RFM95W<LinuxSpiDevice>::SpreadFactor::SF_11;
    else if (spreadfactor == "12")
        this->lora_spread_factor = RFM95W<LinuxSpiDevice>::SpreadFactor::SF_12;
    else
        this->lora_spread_factor = RFM95W<LinuxSpiDevice>::SpreadFactor::SF_12;    

    // Apply LoRa bandwidth setting with default 125k.
    std::string bandwidth = cfgparser.options["lora_bandwidth"];
    if (bandwidth == "7.8")
        this->lora_bandwidth = RFM95W<LinuxSpiDevice>::Bandwidth::BW_7_8K;
    else if (bandwidth == "10.4")
        this->lora_bandwidth = RFM95W<LinuxSpiDevice>::Bandwidth::BW_10_4K;
    else if (bandwidth == "15.6")
        this->lora_bandwidth = RFM95W<LinuxSpiDevice>::Bandwidth::BW_15_6K;
    else if (bandwidth == "20.8")
        this->lora_bandwidth = RFM95W<LinuxSpiDevice>::Bandwidth::BW_20_8K;
    else if (bandwidth == "31.25")
        this->lora_bandwidth = RFM95W<LinuxSpiDevice>::Bandwidth::BW_31_25K;
    else if (bandwidth == "41.7")
        this->lora_bandwidth = RFM95W<LinuxSpiDevice>::Bandwidth::BW_41_7K;
    else if (bandwidth == "62.5")
        this->lora_bandwidth = RFM95W<LinuxSpiDevice>::Bandwidth::BW_62_5K;
    else if (bandwidth == "125")
        this->lora_bandwidth = RFM95W<LinuxSpiDevice>::Bandwidth::BW_125K;
    else if (bandwidth == "250")
        this->lora_bandwidth = RFM95W<LinuxSpiDevice>::Bandwidth::BW_250K;
    else if (bandwidth == "500")
        this->lora_bandwidth = RFM95W<LinuxSpiDevice>::Bandwidth::BW_500K;
    else
        this->lora_bandwidth = RFM95W<LinuxSpiDevice>::Bandwidth::BW_500K;

    // Apply LoRa coding rate with default 48.
    std::string codingrate = cfgparser.options["lora_coding_rate"];
    if (codingrate == "45")
        this->lora_coding_rate = RFM95W<LinuxSpiDevice>::CodingRate::CR_45;
    else if (codingrate == "46")
        this->lora_coding_rate = RFM95W<LinuxSpiDevice>::CodingRate::CR_46;
    else if (codingrate == "47")
        this->lora_coding_rate = RFM95W<LinuxSpiDevice>::CodingRate::CR_47;
    else if (codingrate == "48")
        this->lora_coding_rate = RFM95W<LinuxSpiDevice>::CodingRate::CR_48;
    else
        this->lora_coding_rate = RFM95W<LinuxSpiDevice>::CodingRate::CR_48;

    // Apply LoRa header mode with implicit as default.
    if (cfgparser.options["lora_header_mode"] == "explicit")
        this->lora_header_mode = RFM95W<LinuxSpiDevice>::HeaderMode::HM_EXPLICIT;
    else
        this->lora_header_mode = RFM95W<LinuxSpiDevice>::HeaderMode::HM_IMPLICIT;

    std::stringstream(cfgparser.options["lora_sync_word"]) >> this->lora_sync_word;
    std::stringstream(cfgparser.options["lora_enable_crc"]) >> this->lora_enable_crc;

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
