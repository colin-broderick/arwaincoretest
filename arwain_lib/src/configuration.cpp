#include <arwain/config_parser.hpp>

#include <arwain/vector3.hpp>

#include "arwain/configuration.hpp"
#include "arwain/arwain.hpp"

/** \brief Create a configuration object based on supplied filename.
 * \param filename The location of the configuration file to read.
 * \return int return code.
 */
arwain::ReturnCode arwain::Configuration::read_from_file()
{
    ConfigParser cfgparser{this->config_file};

    // We want to fail out if the model file cannot be found.
    this->inference_model_path = cfgparser.read_option<std::string>("inference/inference_model_path");
    if (!std::filesystem::exists(this->inference_model_path))
    {
        return arwain::ReturnCode::NoInferenceFile;
    }

    this->pos_to_publish = cfgparser.read_option<std::string>("pos_to_publish");

    this->force_z_zero = cfgparser.read_option<bool>("uwb/force_z_zero");

    // Read all options into the configuration object.
    this->active_threshold = cfgparser.read_option<double>("stance/active_threshold");
    this->walking_threshold = cfgparser.read_option<double>("stance/walking_threshold");
    this->running_threshold = cfgparser.read_option<double>("stance/running_threshold");
    this->crawling_threshold = cfgparser.read_option<double>("stance/crawling_threshold");
    this->climbing_threshold = cfgparser.read_option<double>("stance/climbing_threshold");
    this->gravity = cfgparser.read_option<double>("gravity");
    this->struggle_threshold = cfgparser.read_option<double>("stance/struggle_threshold");
    this->freefall_sensitivity = cfgparser.read_option<double>("stance/freefall_sensitivity");
    
    this->hybrid_position_compute = cfgparser.read_option<bool>("hybrid_position/position");
    this->hybrid_heading_compute = cfgparser.read_option<bool>("hybrid_position/heading");
    this->hybrid_position_gain = cfgparser.read_option<double>("hybrid_position/gain");

    this->accel1_bias = Vector3::from_array(cfgparser.read_option<std::array<double, 3>>("imu1/calibration/accel_bias"));
    this->accel2_bias = Vector3::from_array(cfgparser.read_option<std::array<double, 3>>("imu2/calibration/accel_bias"));
    this->accel3_bias = Vector3::from_array(cfgparser.read_option<std::array<double, 3>>("imu3/calibration/accel_bias"));

    this->accel1_scale = Vector3::from_array(cfgparser.read_option<std::array<double, 3>>("imu1/calibration/accel_scale"));
    this->accel2_scale = Vector3::from_array(cfgparser.read_option<std::array<double, 3>>("imu2/calibration/accel_scale"));
    this->accel3_scale = Vector3::from_array(cfgparser.read_option<std::array<double, 3>>("imu3/calibration/accel_scale"));

    this->gyro1_bias = Vector3::from_array(cfgparser.read_option<std::array<double, 3>>("imu1/calibration/gyro_bias"));
    this->gyro2_bias = Vector3::from_array(cfgparser.read_option<std::array<double, 3>>("imu2/calibration/gyro_bias"));
    this->gyro3_bias = Vector3::from_array(cfgparser.read_option<std::array<double, 3>>("imu3/calibration/gyro_bias"));

    this->mag_bias = Vector3::from_array(cfgparser.read_option<std::array<double, 3>>("magnetometer/calibration/bias"));
    this->mag_scale = Vector3::from_array(cfgparser.read_option<std::array<double, 3>>("magnetometer/calibration/scale"));

    //TODO: Convert mag_scale_xy (global) to a Vector or matrix or something suitable
    auto mag_scale_xy_local = cfgparser.read_option<std::array<double, 3>>("magnetometer/calibration/cross_scale");
    this->mag_scale_xy = mag_scale_xy_local[0];
    this->mag_scale_xz = mag_scale_xy_local[1];
    this->mag_scale_yz = mag_scale_xy_local[2];

    this->use_magnetometer = cfgparser.read_option<bool>("orientation/use_magnetometer");
    this->log_magnetometer = cfgparser.read_option<bool>("magnetometer/log_magnetometer");
    this->use_ips = cfgparser.read_option<bool>("indoor_positioning_system/use_ips");
    this->nn_vel_weight_confidence = cfgparser.read_option<double>("inference/nn_vel_weight_confidence");
    this->madgwick_beta = cfgparser.read_option<double>("orientation/madgwick/beta");
    this->madgwick_beta_conv = cfgparser.read_option<double>("orientation/madgwick/beta_conv");
    this->orientation_filter = cfgparser.read_option<std::string>("orientation/filter");
    this->altimeter_z_accel_stdev = cfgparser.read_option<double>("altimeter/calibration/altimeter_z_accel_stdev");
    this->pressure_altitude_stdev = cfgparser.read_option<double>("altimeter/calibration/pressure_altitude_stdev");
    this->use_rs2 = cfgparser.read_option<bool>("intel_rs2/use_rs2");

    this->uubla_baud_rate = cfgparser.read_option<int>("uwb/serial_port/baud_rate");
    this->uubla_serial_port = cfgparser.read_option<std::string>("uwb/serial_port/address");

    this->sea_level_pressure = cfgparser.read_option<double>("altimeter/calibration/sea_level_pressure");
    this->imu1_bus = cfgparser.read_option<std::string>("imu1/i2c/bus");
    this->imu2_bus = cfgparser.read_option<std::string>("imu2/i2c/bus");
    this->imu3_bus = cfgparser.read_option<std::string>("imu3/i2c/bus");
    this->imu1_address = cfgparser.read_option<int>("imu1/i2c/address");
    this->imu2_address = cfgparser.read_option<int>("imu2/i2c/address");
    this->imu3_address = cfgparser.read_option<int>("imu3/i2c/address");
    this->magn_address = cfgparser.read_option<int>("magnetometer/i2c/address");
    this->magn_bus = cfgparser.read_option<std::string>("magnetometer/i2c/bus");
    this->pressure_address = cfgparser.read_option<int>("pressure_sensor/i2c/address");
    this->pressure_bus = cfgparser.read_option<std::string>("pressure_sensor/i2c/bus");
    this->lora_address = cfgparser.read_option<std::string>("lora/spi/address");
    this->node_id = cfgparser.read_option<int>("node_id");
    this->altitude_filter_weight = cfgparser.read_option<double>("altimeter/calibration/altitude_filter_weight");
    this->pressure_offset = cfgparser.read_option<double>("pressure_sensor/calibration/bias");

    this->pressure_offset = cfgparser.read_option<double>("altimeter/calibration/pressure_altitude_stdev");
    this->correct_with_yaw_diff = cfgparser.read_option<bool>("orientation/correct_with_yaw_diff");

    // UWB options.
    this->use_uwb_positioning = cfgparser.read_option<bool>("uwb/use_uwb_positioning");

    // Apply LoRa settings
    this->lora_tx_power = cfgparser.read_option<int>("lora/radio/tx_power");
    this->lora_packet_frequency = cfgparser.read_option<int>("lora/radio/packet_frequency");


    // Apply LoRa radio frequency setting with default 868 MHz.
    std::string rf = cfgparser.read_option<std::string>("lora/radio/rf_frequency");
    if (rf == "433")
        this->lora_rf_frequency = RFM95W<LinuxSpiDevice>::Frequency::FREQ_433;
    else if (rf == "868")
        this->lora_rf_frequency = RFM95W<LinuxSpiDevice>::Frequency::FREQ_868;
    else if (rf == "915")
        this->lora_rf_frequency = RFM95W<LinuxSpiDevice>::Frequency::FREQ_915;
    else
        this->lora_rf_frequency = RFM95W<LinuxSpiDevice>::Frequency::FREQ_868;

    // Apply LoRa spread factor setting with default 12.
    std::string spreadfactor = cfgparser.read_option<std::string>("lora/radio/spread_factor");
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
    std::string bandwidth = cfgparser.read_option<std::string>("lora/radio/bandwidth");
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
    std::string codingrate = cfgparser.read_option<std::string>("lora/radio/coding_rate");
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
    std::string headermode = cfgparser.read_option<std::string>("lora/radio/header_mode");
    if (headermode == "explicit")
        this->lora_header_mode = RFM95W<LinuxSpiDevice>::HeaderMode::HM_EXPLICIT;
    else
        this->lora_header_mode = RFM95W<LinuxSpiDevice>::HeaderMode::HM_IMPLICIT;

    this->lora_sync_word = cfgparser.read_option<int>("lora/radio/sync_word");
    this->lora_enable_crc = cfgparser.read_option<bool>("lora/radio/enable_crc");

    if (this->log_to_stdout)
    {
        std::cout << "Configuration file read successfully\n";
    }

    return arwain::ReturnCode::Success;
}

arwain::Configuration::Configuration(const arwain::InputParser& input)
{    
    // Enable/disable stdout logging.
    if (input.contains("--lstd"))
    {
        this->log_to_stdout = 1;
    }

    // Disable/enable velocity inference.
    if (input.contains("--noinf"))
    {
        this->no_inference = 1;
    }

    // Disable/enable LoRa transmission.
    if (input.contains("--nolora"))
    {
        this->no_lora = 1;
    }

    if (input.contains("--nopressure"))
    {
        this->no_pressure = 1;
    }

    // Disable/enable IMU.
    if (input.contains("--noimu"))
    {
        this->no_imu = 1;
    }

    // If alternate configuration file supplied, read it instead of default.
    if (input.contains("--conf"))
    {
        this->config_file = input.get_cmd_option("--conf");
    }
}
