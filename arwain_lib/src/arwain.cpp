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

#if USE_UUBLA
#include "arwain/uwb_reader.hpp"
#endif

#include "arwain/tests.hpp"
#include "arwain/utils.hpp"
#include "arwain/thread.hpp"
#include "arwain/arwain.hpp"
#include "arwain/sensor_manager.hpp"
#include "arwain/velocity_prediction.hpp"
#include "arwain/transmit_lora.hpp"
#include "arwain/std_output.hpp"
#include "arwain/indoor_positioning_wrapper.hpp"
#include "arwain/altimeter.hpp"
#include "arwain/logger.hpp"
#include "arwain/calibration.hpp"
#include "arwain/command_line.hpp"
#include "arwain/std_output.hpp"
#include "arwain/global_buffer.hpp"
#include "arwain/floor_tracker.hpp"

#include "input_parser.hpp"
#include "iim42652.hpp"
#include "madgwick.hpp"
#include "efaroe.hpp"
#include "lis3mdl.hpp"
#include "bmp384.hpp"
#include "new_madgwick_FusionAhrs.h"
#include "new_madgwick_FusionBias.h"

// General configuration data.
namespace arwain
{
    double yaw_offset = 0;
    arwain::Configuration config;
    std::string folder_date_string;
    std::string folder_date_string_suffix;
    arwain::Logger error_log;
    unsigned int velocity_inference_rate = 20;
    RollingAverage rolling_average_accel_z_for_altimeter{static_cast<int>(static_cast<double>(arwain::Intervals::ALTIMETER_INTERVAL)/1000.0*200)}; // TODO 200 is IMU sample rate, remove magic number
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

/** \brief If the input parser contains -name and a parameter to go with it, sets the global 
 * arwain::folder_date_string_suffix to the value of the parameter. If the input parser does
 * not contain -name, the folder_date_string_suffix is set to an empty string.
 * \param input An InputParser object which may or may not contain the -name parameter.
 */
void arwain::setup_log_folder_name_suffix(const InputParser& input)
{
    if (input.contains("-name"))
    {
        arwain::folder_date_string_suffix = input.get_cmd_option(("-name"));
    }
    else
    {
        arwain::folder_date_string_suffix = "";
    }
}

/** \brief Creates the ARWAIN job threads and then blocks until those threads are ended by setting
 * their modes to arwain::OperatingMode::Terminate.
 * \return ARWAIN return code indiciating success or failure.
*/
arwain::ReturnCode arwain::execute_jobs()
{
    // Start worker threads.
    SensorManager sensor_manager;                            // Reading IMU data, updating orientation filters.
    PositionVelocityInference position_velocity_inference;  // Velocity and position inference.
    StanceDetection stance_detection;                       // Stance, freefall, entanglement detection.
    StatusReporting status_reporting;                       // LoRa packet transmissions.
    DebugPrints debug_prints;
    Altimeter altimeter;                                    // Uses the BMP384 sensor to determine altitude.
    IndoorPositioningSystem indoor_positioning_system;      // Floor, stair, corner snapping.
    #if USE_UUBLA
    UublaWrapper uubla_wrapper;                                   // Enable this node to operate as an UUBLA master node.
    #endif
    #if USE_REALSENSE
    CameraController camera_controller;
    #endif
    ArwainCLI arwain_cli;                                   // Simple command line interface for runtime mode switching.

    while (arwain_cli.get_mode() != arwain::OperatingMode::Terminate)
    {
        sleep_ms(100);
    }

    // Set pointers ...
    status_reporting.set_stance_detection_pointer(stance_detection);
    debug_prints.set_stance_detection_pointer(stance_detection);
    arwain_cli.set_velocity_inference_pointer(position_velocity_inference);

    std::cout << "sensor_manager.join(); " << std::endl;
    sensor_manager.join();
    std::cout << "position_velocity_inference.join(); " << std::endl;
    position_velocity_inference.join();
    std::cout << "stance_detection.join(); " << std::endl;
    stance_detection.join();
    std::cout << "status_reporting.join(); " << std::endl;
    status_reporting.join();
    std::cout << "debug_prints.join(); " << std::endl;
    debug_prints.join();
    std::cout << "altimeter.join(); " << std::endl;
    altimeter.join();
    std::cout << "indoor_positioning_system.join(); " << std::endl;
    indoor_positioning_system.join();
    #if USE_UUBLA
    uubla_wrapper.join();
    #endif
    #if USE_REALSENSE
    camera_controller.join();
    #endif
    std::cout << "arwain_cli.join();" << std::endl;
    arwain_cli.join();
    std::cout << "All threads joined" << std::endl;

    return arwain::ReturnCode::Success;
}

arwain::ReturnCode arwain::calibrate_magnetometers()
{
    StandAloneModeRegistrar mode_registrar;
    LIS3MDL<I2CDEVICEDRIVER> magnetometer{arwain::config.magn_address, arwain::config.magn_bus};
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

    arwain::config.replace("mag_bias_x", bias[0]);
    arwain::config.replace("mag_bias_y", bias[1]);
    arwain::config.replace("mag_bias_z", bias[2]);
    arwain::config.replace("mag_scale_x", scale[0][0]);
    arwain::config.replace("mag_scale_y", scale[1][1]);
    arwain::config.replace("mag_scale_z", scale[2][2]);
    arwain::config.replace("mag_scale_xy", scale[0][1]);
    arwain::config.replace("mag_scale_xz", scale[0][2]);
    arwain::config.replace("mag_scale_yz", scale[1][2]);

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

    IIM42652<I2CDEVICEDRIVER> imu1{arwain::config.imu1_address, arwain::config.imu1_bus};
    std::cout << "Calibrating gyroscope on " << imu1.get_bus() << " at 0x" << std::hex << imu1.get_address() << "; please wait\n";
    GyroscopeCalibrator calibrator1;
    while (!calibrator1.is_converged())
    {
        calibrator1.feed(imu1.read_imu().gyro);
        sleep_ms(5);
    }
    results = calibrator1.get_params();
    arwain::config.replace("gyro1_bias_x", results.x);
    arwain::config.replace("gyro1_bias_y", results.y);
    arwain::config.replace("gyro1_bias_z", results.z);

    IIM42652<I2CDEVICEDRIVER> imu2{arwain::config.imu1_address, arwain::config.imu1_bus};
    std::cout << "Calibrating gyroscope on " << imu2.get_bus() << " at 0x" << std::hex << imu2.get_address() << "; please wait\n";
    GyroscopeCalibrator calibrator2;
    while (!calibrator2.is_converged())
    {
        calibrator2.feed(imu2.read_imu().gyro);
        sleep_ms(5);
    }
    results = calibrator2.get_params();
    arwain::config.replace("gyro2_bias_x", results.x);
    arwain::config.replace("gyro2_bias_y", results.y);
    arwain::config.replace("gyro2_bias_z", results.z);

    IIM42652<I2CDEVICEDRIVER> imu3{arwain::config.imu1_address, arwain::config.imu1_bus};
    std::cout << "Calibrating gyroscope on " << imu3.get_bus() << " at 0x" << std::hex << imu3.get_address() << "; please wait\n";
    GyroscopeCalibrator calibrator3;
    while (!calibrator3.is_converged())
    {
        calibrator3.feed(imu3.read_imu().gyro);
        sleep_ms(5);
    }
    results = calibrator3.get_params();
    arwain::config.replace("gyro3_bias_x", results.x);
    arwain::config.replace("gyro3_bias_y", results.y);
    arwain::config.replace("gyro3_bias_z", results.z);

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

    IIM42652<I2CDEVICEDRIVER> imu1{arwain::config.imu1_address, arwain::config.imu1_bus};
    IIM42652<I2CDEVICEDRIVER> imu2{arwain::config.imu2_address, arwain::config.imu2_bus};
    IIM42652<I2CDEVICEDRIVER> imu3{arwain::config.imu3_address, arwain::config.imu3_bus};

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
    arwain::config.replace("accel1_bias_x", bias_1.x);
    arwain::config.replace("accel1_bias_y", bias_1.y);
    arwain::config.replace("accel1_bias_z", bias_1.z);
    arwain::config.replace("accel1_scale_x", scale_1.x);
    arwain::config.replace("accel1_scale_y", scale_1.y);
    arwain::config.replace("accel1_scale_z", scale_1.z);

    arwain::config.replace("accel2_bias_x", bias_2.x);
    arwain::config.replace("accel2_bias_y", bias_2.y);
    arwain::config.replace("accel2_bias_z", bias_2.z);
    arwain::config.replace("accel2_scale_x", scale_2.x);
    arwain::config.replace("accel2_scale_y", scale_2.y);
    arwain::config.replace("accel2_scale_z", scale_2.z);

    arwain::config.replace("accel3_bias_x", bias_3.x);
    arwain::config.replace("accel3_bias_y", bias_3.y);
    arwain::config.replace("accel3_bias_z", bias_3.z);
    arwain::config.replace("accel3_scale_x", scale_3.x);
    arwain::config.replace("accel3_scale_y", scale_3.y);
    arwain::config.replace("accel3_scale_z", scale_3.z);

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
        EventManager::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
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
        {arwain::ReturnCode::NoInferenceXML, "Inference model XML file not found."},
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
    InputParser input{argc, argv};

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
    }
    
    if (input.contains("--fulltest") || input.contains("-f"))
    {
        std::cout << "Entering interactive test mode\n";
        return arwain::interactive_test();
    }

    // Start IMU test mode. This returns so the program will quit when the test is stopped.
    else if (input.contains("--testimu"))
    {
        ret = arwain::test_imu("/dev/i2c-1", 0x68);
    }
    else if (input.contains("-testlorarx"))
    {
        ret = arwain::test_lora_rx();
    }
    else if (input.contains("-testloratx"))
    {
        ret = arwain::test_lora_tx();
    }
    else if (input.contains("-calibm"))
    {
        ret = arwain::calibrate_magnetometers();
    }
    else if (input.contains("-testpressure"))
    {
        ret = arwain::test_pressure();
    }
    else if (input.contains("-testori"))
    {
        int rate;
        const char *rate_str = input.get_cmd_option("-testori").c_str();
        rate = std::atoi(rate_str);
        ret = arwain::test_ori(rate);
    }
    else if (input.contains("-rerunori"))
    {
        ret = arwain::rerun_orientation_filter(input.get_cmd_option("-rerunori"));
    }
    else if (input.contains("-rerunfloor"))
    {
        ret = arwain::rerun_floor_tracker(input.get_cmd_option("-rerunfloor"));
    }

    // Perform quick calibration of gyroscopes and write to config file.
    else if (input.contains("-calibg"))
    {
        ret = arwain::calibrate_gyroscopes_offline();
    }

    // Perform quick calibration of gyroscopes and write to config file.
    else if (input.contains("-caliba"))
    {
        ret = arwain::calibrate_accelerometers_simple();
    }
    else if (input.contains("-testmag"))
    {
        #if USE_ROS
        ret = arwain::test_mag(argc, argv);
        #else
        ret = arwain::test_mag();
        #endif
    }
    #if USE_UUBLA
    else if (input.contains("-testuubla"))
    {
        ret = arwain::test_uubla_integration();
    }
    #endif

    else
    {
        // Attempt to calibrate the gyroscope before commencing other activities.
        if (input.contains("-calib"))
        {
            arwain::calibrate_gyroscopes_offline();
            arwain::config = arwain::Configuration{input}; // Reread the config file as it has now changed.
        }
        arwain::setup_log_folder_name_suffix(input);
        ret = arwain::execute_jobs();
    }

    return ret;
}
