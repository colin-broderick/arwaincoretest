#include <csignal>
#include <iostream>
#include <filesystem>
#include <iomanip>
#include <map>
#include <thread>
#include <cmath>
#include <tuple>

#if USE_ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#endif

#if USE_REALSENSE
#include "arwain/realsense.hpp"
#endif

#include "arwain/uwb_reader.hpp"
#include "arwain/tests.hpp"
#include "arwain/utils.hpp"
#include "arwain/arwain.hpp"
#include "arwain/sensor_manager.hpp"
#include "arwain/velocity_prediction.hpp"
#include "arwain/transmit_lora.hpp"
#include "arwain/std_output.hpp"
#include "arwain/indoor_positioning_wrapper.hpp"
#include "arwain/altimeter.hpp"
#include "arwain/calibration.hpp"
#include "arwain/command_line.hpp"
#include "arwain/std_output.hpp"
#include "arwain/global_buffer.hpp"
#include "arwain/floor_tracker.hpp"
#include "arwain/events.hpp"
#include "arwain/ws_messenger.hpp"

#include <arwain/logger.hpp>
#include <arwain/input_parser.hpp>
#include <arwain/devices/iim42652.hpp>
#include <arwain/devices/bmp384.hpp>
#include <arwain/devices/lis3mdl.hpp>
#include <arwain/orientation/madgwick.hpp>
#include <arwain/orientation/efaroe.hpp>
#include <arwain/orientation/new_madgwick_FusionAhrs.h>
#include <arwain/orientation/new_madgwick_FusionBias.h>

#include "arwain/i2c_interface.hpp"
#include "arwain/hybrid_positioner.hpp"

// General configuration data.
namespace arwain
{
    double yaw_offset = 0;
    arwain::Configuration config;
    std::string folder_date_string;
    std::string folder_date_string_suffix;
    arwain::Logger error_log;
    unsigned int velocity_inference_rate = 20;
    RollingAverage rolling_average_accel_z_for_altimeter{static_cast<int>(static_cast<double>(arwain::Intervals::ALTIMETER_INTERVAL)/1000.0*(1000/arwain::Intervals::IMU_READING_INTERVAL))};
    ActivityMetric activity_metric{200, 20};
}

// Shared data buffers; mutex locks must be used when accessing.
namespace arwain::Buffers
{
    GlobalBuffer<ImuData, arwain::BufferSizes::IMU_BUFFER_LEN> IMU_BUFFER;
    GlobalBuffer<ImuData, arwain::BufferSizes::IMU_BUFFER_LEN> IMU_WORLD_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::VELOCITY_BUFFER_LEN> VELOCITY_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::POSITION_BUFFER_LEN> POSITION_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::MAG_BUFFER_LEN> MAG_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::MAG_BUFFER_LEN> MAG_WORLD_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::IPS_BUFFER_LEN> IPS_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::PRESSURE_BUFFER_LEN> PRESSURE_BUFFER;
    GlobalBuffer<EulerOrientation, arwain::BufferSizes::ORIENTATION_BUFFER_LEN> EULER_ORIENTATION_BUFFER;
    GlobalBuffer<Quaternion, arwain::BufferSizes::ORIENTATION_BUFFER_LEN> QUAT_ORIENTATION_BUFFER;
    GlobalBuffer<Quaternion, arwain::BufferSizes::MAG_ORIENTATION_BUFFER_LEN> MAG_ORIENTATION_BUFFER;
    GlobalBuffer<double, arwain::BufferSizes::MAG_EULER_BUFFER_LEN> MAG_EULER_BUFFER;
}

std::array<double, 3> vec_to_array3(std::vector<double> in_vector)
{
    if (in_vector.size() == 3)
    {
        return {
            in_vector[0],
            in_vector[1],
            in_vector[2],
        };
    }
    else
    {
        throw std::exception{};
    }
}

/** \brief Reads the position.txt file from the given location, and 
 * runs the floor tracking algorithm against that data.
 */
arwain::ReturnCode arwain::rerun_floor_tracker(const std::string& data_location)
{
    std::cout << "Reprocessing floor tracker on dataset \"" << data_location << "\"" << std::endl;

    int64_t t;
    double x, y, z;

    std::ifstream position_input{data_location + "/position.txt"};

    std::ofstream position_output{"pos_out.txt"};

    // Clear data file headers.
    std::string position_line;
    std::getline(position_input, position_line);

    arwain::FloorTracker flt{5, 0.10, 0.20};

    while(std::getline(position_input, position_line))
    {
        // Get IMU data
        std::istringstream pstream(position_line);
        pstream >> t >> x >> y >> z;
        flt.update({x, y, z});
        position_output << flt.tracked_position.x << " " << flt.tracked_position.y << " " << flt.tracked_position.z << "\n";
    }

    position_output.close();

    std::cout << "Reprocessing complete" << std::endl;

    return arwain::ReturnCode::Success;
}

/** \brief Reads the acce.txt and gyro.txt files from the given location, 
 * and reruns the selected orientation filter(s) against that data.
 */
arwain::ReturnCode arwain::rerun_orientation_filter(const std::string& data_location)
{
    std::cout << "Reprocessing orientation filter on dataset \"" << data_location << "\"" << std::endl;

    int64_t t;
    double ax, ay, az, gx, gy, gz;

    std::ifstream acce{data_location + "/acce.txt"};
    std::ifstream gyro{data_location + "/gyro.txt"};

    std::ofstream slow_file{"slow_file.txt"};
    std::ofstream fast_file{"fast_file.txt"};
    std::ofstream fusion_file{"fusion_file.txt"};

    // Clear data file headers.
    std::string acce_line;
    std::string gyro_line;
    std::getline(acce, acce_line);
    std::getline(gyro, gyro_line);

    arwain::Madgwick slow_filter{1000.0/arwain::Intervals::IMU_READING_INTERVAL, 0.1};
    arwain::Madgwick fast_filter{1000.0/arwain::Intervals::IMU_READING_INTERVAL, 0.9};
    FusionBias fusionBias;
    FusionAhrs fusionAhrs;
    FusionBiasInitialise(&fusionBias, 0.5, 0.005);
    FusionAhrsInitialise(&fusionAhrs, 0.5);

    std::vector<double> slow_yaw;
    std::vector<double> fast_yaw;
    std::vector<double> fusion_yaw;

    while(std::getline(acce, acce_line) && std::getline(gyro, gyro_line))
    {
        // Get IMU data
        std::istringstream astream(acce_line);
        std::istringstream gstream(gyro_line);
        astream >> t >> ax >> ay >> az;
        gstream >> t >> gx >> gy >> gz;

        slow_filter.update(t, gx, gy, gz, ax, ay, az);
        fast_filter.update(t, gx, gy, gz, ax, ay, az);

        FusionAhrsUpdateWithoutMagnetometer(
            &fusionAhrs, 
            {(float)(gx*180.0/3.14159), (float)(gy*180.0/3.14159), (float)(gz*180.0/3.14159)},
            {(float)(ax/9.81), (float)(ay/9.81), (float)(az/9.81)},
            0.005
        );

        if (slow_yaw.empty())
        {
            slow_yaw.push_back(slow_filter.get_yaw());
        }
        else
        {
            slow_yaw.push_back(unwrap_phase_degrees(slow_filter.get_yaw(), slow_yaw.back()));
        }

        if (fast_yaw.empty())
        {
            fast_yaw.push_back(fast_filter.get_yaw());
        }
        else
        {
            fast_yaw.push_back(unwrap_phase_degrees(fast_filter.get_yaw(), fast_yaw.back()));
        }

        if (fusion_yaw.empty())
        {
            double yaw = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs)).angle.yaw;
            fusion_yaw.push_back(yaw);
        }
        else
        {
            double yaw = FusionQuaternionToEulerAngles(FusionAhrsGetQuaternion(&fusionAhrs)).angle.yaw;
            fusion_yaw.push_back(unwrap_phase_degrees(yaw, fusion_yaw.back()));
        }
    }

    for (auto& elem : slow_yaw)
    {
        slow_file << elem << "\n";
    }
    for (auto& elem : fast_yaw)
    {
        fast_file << elem << "\n";
    }
    for (auto& elem : fusion_yaw)
    {
        fusion_file << elem << "\n";
    }

    slow_file.close();
    fast_file.close();
    fusion_file.close();

    std::cout << "Reprocessing complete" << std::endl;

    return arwain::ReturnCode::Success;
}

void arwain::setup_log_directory()
{
    if (arwain::folder_date_string_suffix == "nullname")
    {
        arwain::folder_date_string_suffix = "";
    }
    if (arwain::folder_date_string_suffix != "")
    {
        arwain::folder_date_string = "./data_" + date_time_string() + "_" + arwain::folder_date_string_suffix;
    }
    else
    {
        arwain::folder_date_string = "./data_" + date_time_string();
    }
    if (!std::filesystem::is_directory(arwain::folder_date_string))
    {
        std::filesystem::create_directory(arwain::folder_date_string);
    }
    std::filesystem::copy(arwain::config.config_file, arwain::folder_date_string + "/config.conf");

    // Close the current error log, if there is one, and start a new one.
    if (arwain::error_log.is_open())
    {
        arwain::error_log.close();
    }
    arwain::error_log.open(arwain::folder_date_string + "/ERRORS.txt");
    arwain::error_log << "time event" << "\n";
}

/** \brief If the input parser contains --name and a parameter to go with it, sets the global 
 * arwain::folder_date_string_suffix to the value of the parameter. If the input parser does
 * not contain --name, the folder_date_string_suffix is set to an empty string.
 * \param input An InputParser object which may or may not contain the -name parameter.
 */
void arwain::setup_log_folder_name_suffix(const arwain::InputParser& input)
{
    if (input.contains("--name"))
    {
        arwain::folder_date_string_suffix = input.get_cmd_option(("--name"));
    }
    else
    {
        arwain::folder_date_string_suffix = "";
    }
}

// std::unique_ptr<WsMessenger> ws_messenger;
std::unique_ptr<HybridPositioner> hybrid_positioner;
std::unique_ptr<UublaWrapper> uubla_wrapper;
std::unique_ptr<SensorManager> sensor_manager;
std::unique_ptr<PositionVelocityInference> position_velocity_inference;
std::unique_ptr<StanceDetection> stance_detection;                       // Stance, freefall, entanglement detection.
std::unique_ptr<Altimeter> altimeter;                                    // Uses the BMP384 sensor to determine altitude.
std::unique_ptr<IndoorPositioningSystem> indoor_positioning_system;      // Floor, stair, corner snapping.
std::unique_ptr<ArwainCLI> arwain_cli;                                   // Simple command line interface for runtime mode switching.
std::unique_ptr<StatusReporting> status_reporting;                       // LoRa packet transmissions.
// std::unique_ptr<CameraController> camera_controller;

/** \brief Creates the ARWAIN job threads and then blocks until those threads are ended by setting
 * their modes to arwain::OperatingMode::Terminate.
 * \return ARWAIN return code indiciating success or failure.
*/
arwain::ReturnCode arwain::execute_jobs()
{
    // Start worker threads.
    // ws_messenger = std::make_unique<WsMessenger>();                              // There should never have been a WS server in this code base!
    sensor_manager = std::make_unique<SensorManager>();                             // Reading IMU data, updating orientation filters.
    position_velocity_inference = std::make_unique<PositionVelocityInference>();    // Velocity and position inference.
    stance_detection = std::make_unique<StanceDetection>();;                        // Stance, freefall, entanglement detection.
    altimeter = std::make_unique<Altimeter>();                                      // Uses the BMP384 sensor to determine altitude.
    indoor_positioning_system = std::make_unique<IndoorPositioningSystem>();        // Floor, stair, corner snapping.
    uubla_wrapper = std::make_unique<UublaWrapper>();                               // Enable this node to operate as an UUBLA master node.
    // status_reporting = std::make_unique<StatusReporting>();                      // LoRa packet transmissions.

    // If neither of the hybrid subsystems are enabled, no point loading the hybridizer.
    if (arwain::config.hybrid_heading_compute || arwain::config.hybrid_position_compute)
    {
        hybrid_positioner = std::make_unique<HybridPositioner>();
    }

    #if USE_REALSENSE
    camera_controller = std::make_unique<CameraController>(); 
    #endif
    arwain_cli = std::make_unique<ArwainCLI>();                                   // Simple command line interface for runtime mode switching.

    while (arwain_cli->get_mode() != arwain::OperatingMode::Terminate)
    {
        sleep_ms(100);
    }

    return arwain::ReturnCode::Success;
}

arwain::ReturnCode arwain::calibrate_magnetometers()
{
    StandAloneModeRegistrar mode_registrar;
    LIS3MDL<LinuxSmbusI2CDevice> magnetometer{arwain::config.magn_address, arwain::config.magn_bus};
    MagnetometerCalibrator calibrator;
    std::cout << "About to start magnetometer calibration." << std::endl;
    std::cout << "Move the device through all orientations; press Ctrl+C when done." << std::endl;
    sleep_ms(3000);
    std::cout << "Calibration started ..." << std::endl;

    while (mode_registrar.get_mode() != arwain::OperatingMode::Terminate)
    {
        calibrator.feed(magnetometer.read());
        sleep_ms(100);
    }
    
    auto [bias, scale] = calibrator.solve();

    std::cout << "Bias: " << bias[0] << " " << bias[1] << " " << bias[2] << "\n";
    std::cout << "Scale:\n";
    std::cout << scale[0][0] << " " << scale[0][1] << " " << scale[0][2] << std::endl;
    std::cout << scale[1][0] << " " << scale[1][1] << " " << scale[1][2] << std::endl;
    std::cout << scale[2][0] << " " << scale[2][1] << " " << scale[2][2] << std::endl;
    std::cout << std::endl;

    arwain::config.replace("magnetometer/calibration/bias", vec_to_array3(bias));

    std::array<double, 3> mag_scale_xyz = {scale[0][0], scale[1][1], scale[2][2]};
    arwain::config.replace("magnetometer/calibration/scale", mag_scale_xyz);
    std::array<double, 3> mag_scale_cross = {scale[0][1], scale[0][2], scale[1][2]};
    arwain::config.replace("magnetometer/calibration/cross_scale", mag_scale_cross);

    std::cout << "Magnetometer calibration complete" << std::endl;
    return arwain::ReturnCode::Success;
}

/** \brief Initializes IMUs based on the address/bus pairs specified in arwain.conf, and
 * calibrates the bias offset for all gyroscopes, then stores the result in arwain.conf.
 * \return ARWAIN return code.
*/
arwain::ReturnCode arwain::calibrate_gyroscopes_offline()
{
    Vector3 results;

    IIM42652<LinuxSmbusI2CDevice> imu1{arwain::config.imu1_address, arwain::config.imu1_bus};
    std::cout << "Calibrating gyroscope on " << imu1.get_bus() << " at 0x" << std::hex << imu1.get_address() << "; please wait\n";
    GyroscopeCalibrator calibrator1;
    while (!calibrator1.is_converged())
    {
        calibrator1.feed(imu1.read_imu().gyro);
        sleep_ms(5);
    }
    results = calibrator1.get_params();
    arwain::config.replace("imu1/calibration/gyro_bias", results.to_array());

    IIM42652<LinuxSmbusI2CDevice> imu2{arwain::config.imu2_address, arwain::config.imu2_bus};
    std::cout << "Calibrating gyroscope on " << imu2.get_bus() << " at 0x" << std::hex << imu2.get_address() << "; please wait\n";
    GyroscopeCalibrator calibrator2;
    while (!calibrator2.is_converged())
    {
        calibrator2.feed(imu2.read_imu().gyro);
        sleep_ms(5);
    }
    results = calibrator2.get_params();
    arwain::config.replace("imu2/calibration/gyro_bias", results.to_array());

    IIM42652<LinuxSmbusI2CDevice> imu3{arwain::config.imu3_address, arwain::config.imu3_bus};
    std::cout << "Calibrating gyroscope on " << imu3.get_bus() << " at 0x" << std::hex << imu3.get_address() << "; please wait\n";
    GyroscopeCalibrator calibrator3;
    while (!calibrator3.is_converged())
    {
        calibrator3.feed(imu3.read_imu().gyro);
        sleep_ms(5);
    }
    results = calibrator3.get_params();
    arwain::config.replace("imu3/calibration/gyro_bias", results.to_array());

    std::cout << std::dec;
    std::cout << "Gyroscope calibration complete.\n";

    return arwain::ReturnCode::Success;
}

std::tuple<Vector3, Vector3> deduce_calib_params(const std::vector<Vector3>& readings)
{
    double x_min = 1e6;
    double x_max = -1e6;
    double y_min = 1e6;
    double y_max = -1e6;
    double z_min = 1e6;
    double z_max = -1e6;
    
    for (const Vector3& vec : readings)
    {
        x_min = vec.x < x_min ? vec.x : x_min;
        x_max = vec.x > x_max ? vec.x : x_max;
        y_min = vec.y < y_min ? vec.y : y_min;
        y_max = vec.y > y_max ? vec.y : y_max;
        z_min = vec.z < z_min ? vec.z : z_min;
        z_max = vec.z > z_max ? vec.z : z_max;
    }
    Vector3 bias_ = {(x_min + x_max) / 2.0, (y_min + y_max) / 2.0, (z_min + z_max) / 2.0};

    // Compute scale correction factors.
    Vector3 delta = {(x_max - x_min) / 2.0, (y_max - y_min) / 2.0, (z_max - z_min) / 2.0};
    double average_delta = (delta.x + delta.y + delta.z)/3.0;
    double scale_x = average_delta / delta.x;
    double scale_y = average_delta / delta.y;
    double scale_z = average_delta / delta.z;
    Vector3 scale_ = {scale_x, scale_y, scale_z};

    return {bias_, scale_};
}

arwain::ReturnCode arwain::calibrate_accelerometers_simple()
{
    std::cout << "Calibrating the following accelerometers:" << std::endl;
    std::cout << "\t" << arwain::config.imu1_bus << " at 0x" << std::hex << arwain::config.imu1_address << std::endl;
    std::cout << "\t" << arwain::config.imu2_bus << " at 0x" << std::hex << arwain::config.imu2_address << std::endl;
    std::cout << "\t" << arwain::config.imu3_bus << " at 0x" << std::hex << arwain::config.imu3_address << std::endl;
    std::cout << std::dec << std::endl;

    IIM42652<LinuxSmbusI2CDevice> imu1{arwain::config.imu1_address, arwain::config.imu1_bus};
    IIM42652<LinuxSmbusI2CDevice> imu2{arwain::config.imu2_address, arwain::config.imu2_bus};
    IIM42652<LinuxSmbusI2CDevice> imu3{arwain::config.imu3_address, arwain::config.imu3_bus};

    AccelerometerCalibrator calib1;
    AccelerometerCalibrator calib2;
    AccelerometerCalibrator calib3;

    std::cout << "Accelerometer calibration starting\n";
    sleep_ms(5000);

    for (int i = 0; i < 6; i++)
    {
        std::cout << "Place the device in a random orientation\n";
        sleep_ms(5000);

        while (!calib1.is_converged() || !calib2.is_converged() || !calib3.is_converged())
        {
            calib1.feed(imu1.read_imu().acce);
            calib2.feed(imu2.read_imu().acce);
            calib3.feed(imu3.read_imu().acce);
        }
        calib1.next_sampling();
        calib2.next_sampling();
        calib3.next_sampling();
    }

    std::cout << "Data collection complete; computing calibration parameters...\n";
    auto [bias_1, scale_1] = calib1.deduce_calib_params();
    auto [bias_2, scale_2] = calib2.deduce_calib_params();
    auto [bias_3, scale_3] = calib3.deduce_calib_params();

    // TODO Detect and remove outliers.

    // Log to config file.
    arwain::config.replace("imu1/calibration/accel_bias", bias_1.to_array());
    arwain::config.replace("imu1/calibration/accel_scale", scale_1.to_array());

    arwain::config.replace("imu2/calibration/accel_bias", bias_2.to_array());
    arwain::config.replace("imu2/calibration/accel_scale", scale_2.to_array());

    arwain::config.replace("imu3/calibration/accel_bias", bias_3.to_array());
    arwain::config.replace("imu3/calibration/accel_scale", scale_3.to_array());

    std::cout << "Calibration complete and logged to config file\n";

    return arwain::ReturnCode::Success;
}

/** \brief Capture the SIGINT signal for clean exit.
 * After calling, the system mode is Terminate, which instructs all threads to clean up and exit.
 * \param signal The signal to capture.
 */
void sigint_handler(int signal)
{
    if (signal == SIGINT)
    {
        std::cout << "\nReceived SIGINT - closing\n" << "\n";
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    }
}

namespace arwain
{
    std::map<arwain::ReturnCode, std::string> ErrorMessages = {
        {arwain::ReturnCode::Success, "Success."},
        {arwain::ReturnCode::FailedIMU, "Could not communicate with IMU."},
        {arwain::ReturnCode::FailedConfiguration, "Configuration failed."},
        {arwain::ReturnCode::FailedMagnetometer, "Could not communicate with magnetometer."},
        {arwain::ReturnCode::NoConfigurationFile, "Could not find configuration file."},
        {arwain::ReturnCode::NoInferenceFile, "Inference model file not found."},
        {arwain::ReturnCode::IMUReadError, "Encountered error reading IMU."},
    };
}

/** \brief ARWAIN entry point. */
arwain::ReturnCode arwain_main(int argc, char **argv)
{
    #if USE_ROS
    ros::init(argc, argv, "arwain_node");
    #endif

    arwain::ReturnCode ret;

    // Prepare keyboard interrupt signal handler to enable graceful exit.
    std::signal(SIGINT, sigint_handler);
    
    // Determine behaviour from command line arguments.
    arwain::InputParser input{argc, argv};

    // Output help text if requested.
    if (input.contains("--help") || input.contains("-h"))
    {
        std::cout << arwain::help_text << std::endl;
        return arwain::ReturnCode::Success;
    }
    if (input.contains("--version") || input.contains("-v"))
    {
        std::cout << "ARWAIN executable version 0.1\n";
        return arwain::ReturnCode::Success;
    }

    // Attempt to read the config file and quit if failed.
    arwain::config = arwain::Configuration{input};
    if ((ret = arwain::config.read_from_file()) != arwain::ReturnCode::Success)
    {
        std::cout << "Got an error when reading config file:\n";
        std::cout << arwain::ErrorMessages[ret] << std::endl;
        return ret;
    }
    
    if (input.contains("--fulltest") || input.contains("-f"))
    {
        std::cout << "Entering interactive test mode\n";
        return arwain::interactive_test();
    }

    // Start IMU test mode. This returns so the program will quit when the test is stopped.
    else if (input.contains("--testimu"))
    {
        ret = arwain::test_imu(input.get_cmd_option("--testimu"), std::stoi(input.get_cmd_option("--addr")));
    }
    else if (input.contains("--testlorarx"))
    {
        ret = arwain::test_lora_rx();
    }
    else if (input.contains("-testloratx"))
    {
        ret = arwain::test_lora_tx();
    }
    else if (input.contains("--calibm"))
    {
        ret = arwain::calibrate_magnetometers();
    }
    else if (input.contains("-testpressure"))
    {
        ret = arwain::test_pressure();
    }
    else if (input.contains("--testori"))
    {
        int rate;
        const char *rate_str = input.get_cmd_option("--testori").c_str();
        rate = std::atoi(rate_str);
        ret = arwain::test_ori(rate);
    }
    else if (input.contains("--rerunori"))
    {
        ret = arwain::rerun_orientation_filter(input.get_cmd_option("--rerunori"));
    }
    else if (input.contains("--rerunfloor"))
    {
        ret = arwain::rerun_floor_tracker(input.get_cmd_option("--rerunfloor"));
    }

    // Perform quick calibration of gyroscopes and write to config file.
    else if (input.contains("--calibg"))
    {
        ret = arwain::calibrate_gyroscopes_offline();
    }

    // Perform quick calibration of gyroscopes and write to config file.
    else if (input.contains("--caliba"))
    {
        ret = arwain::calibrate_accelerometers_simple();
    }
    else if (input.contains("--testmag"))
    {
        #if USE_ROS
        ret = arwain::test_mag(argc, argv);
        #else
        ret = arwain::test_mag();
        #endif
    }
    #if USE_UUBLA
    else if (input.contains("--testuubla"))
    {
        ret = arwain::test_uubla_integration();
    }
    #endif

    else
    {
        // Attempt to calibrate the gyroscope before commencing other activities.
        if (input.contains("--calib"))
        {
            arwain::calibrate_gyroscopes_offline();
            arwain::config = arwain::Configuration{input}; // Reread the config file as it has now changed.
        }
        arwain::setup_log_folder_name_suffix(input);
        ret = arwain::execute_jobs();
    }

    return ret;
}
