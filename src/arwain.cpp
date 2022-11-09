#include <iostream>
#include <filesystem>
#include <iomanip>
#include <thread>
#include <cmath>
#include <tuple>

#include "arwain_thread.hpp"
#include "build_config.hpp"
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
#include "bmp384.hpp"
#include "calibration.hpp"
#include "command_line.hpp"
// #include "uwb_reader.hpp"
#include "uubla.hpp"
#include "global_buffer.hpp"

#if USE_REALSENSE == 1
#include "t265.hpp"
#endif

#if USE_ROS == 1
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#endif

#include "floor_tracker.hpp"
#include "new_madgwick_FusionAhrs.h"
#include "new_madgwick_FusionBias.h"

// General configuration data.
namespace arwain
{
    int shutdown = 0;
    OperatingMode system_mode = arwain::OperatingMode::AutoCalibration;
    double yaw_offset = 0;
    arwain::Configuration config;
    std::string folder_date_string;
    std::string folder_date_string_suffix;
    arwain::Status status;
    arwain::Logger error_log;
    bool reset_position = false;
    bool request_gyro_calib = false;
    bool ready_for_inference = false;
    unsigned int velocity_inference_rate = 20;
    RollingAverage rolling_average_accel_z_for_altimeter{static_cast<int>(static_cast<double>(arwain::Intervals::ALTIMETER_INTERVAL)/1000.0*200)}; // TODO 200 is IMU sample rate, remove magic number
    ActivityMetric activity_metric{200, 20};
    #if USE_UUBLA == 1
    UUBLA::Network* uubla_handle = nullptr;
    #endif
}

// Shared data buffers; mutex locks must be used when accessing.
namespace arwain::Buffers
{
    GlobalBuffer<Vector6, arwain::BufferSizes::IMU_BUFFER_LEN> IMU_BUFFER;
    GlobalBuffer<Vector6, arwain::BufferSizes::IMU_BUFFER_LEN> IMU_WORLD_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::VELOCITY_BUFFER_LEN> VELOCITY_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::POSITION_BUFFER_LEN> POSITION_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::MAG_BUFFER_LEN> MAG_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::MAG_BUFFER_LEN> MAG_WORLD_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::IPS_BUFFER_LEN> IPS_BUFFER;
    GlobalBuffer<Vector3, arwain::BufferSizes::PRESSURE_BUFFER_LEN> PRESSURE_BUFFER;
    GlobalBuffer<euler_orientation_t, arwain::BufferSizes::ORIENTATION_BUFFER_LEN> EULER_ORIENTATION_BUFFER;
    GlobalBuffer<Quaternion, arwain::BufferSizes::ORIENTATION_BUFFER_LEN> QUAT_ORIENTATION_BUFFER;
    GlobalBuffer<Quaternion, arwain::BufferSizes::MAG_ORIENTATION_BUFFER_LEN> MAG_ORIENTATION_BUFFER;
    GlobalBuffer<double, arwain::BufferSizes::MAG_EULER_BUFFER_LEN> MAG_EULER_BUFFER;
}

void sleep_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds{ms});
}

/** \brief Start a Python script which opens a socket and does inference on data we send it. */
static void py_inference()
{   
   if (!arwain::config.no_inference)
   {   
       std::string command = "PYTHONPATH=/opt/intel/openvino/python/python3.7:/opt/intel/openvino/python/python3:/opt/intel/openvino/deployment_tools/model_optimizer:/opt/ros/melodic/lib/python2.7/dist-packages:/opt/intel/openvino/python/python3.7:/opt/intel/openvino/python/python3:/opt/intel/openvino/deployment_tools/model_optimizer InferenceEngine_DIR=/opt/intel/openvino/deployment_tools/inference_engine/share INTEL_OPENVINO_DIR=/opt/intel/openvino OpenCV_DIR=/opt/intel/openvino/opencv/cmake LD_LIBRARY_PATH=/opt/intel/openvino/opencv/lib:/opt/intel/openvino/deployment_tools/ngraph/lib:/opt/intel/opencl:/opt/intel/openvino/deployment_tools/inference_engine/external/hddl/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/gna/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/mkltiny_lnx/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/tbb/lib:/opt/intel/openvino/deployment_tools/inference_engine/lib/armv7l:/opt/ros/melodic/lib:/opt/intel/openvino/opencv/lib:/opt/intel/openvino/deployment_tools/ngraph/lib:/opt/intel/opencl:/opt/intel/openvino/deployment_tools/inference_engine/external/hddl/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/gna/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/mkltiny_lnx/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/tbb/lib:/opt/intel/openvino/deployment_tools/inference_engine/lib/armv7l PATH=/opt/intel/openvino/deployment_tools/model_optimizer:/opt/ros/melodic/bin:/home/pi/.vscode-server/bin/ccbaa2d27e38e5afa3e5c21c1c7bef4657064247/bin:/home/pi/.local/bin:/opt/intel/openvino/deployment_tools/model_optimizer:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games /usr/bin/python3 ./python_utils/ncs2_interface.py " + arwain::config.inference_model_xml + " > /dev/null &";
       system(command.c_str());
   }
}

double unwrap_phase_radians(double new_angle, double previous_angle)
{
    while (new_angle - previous_angle > 3.14159)
    {
        new_angle -= 2.0*3.14159;
    }
    while (new_angle - previous_angle < -3.14159)
    {
        new_angle += 2.0*3.14159;
    }
    return new_angle;
}

double unwrap_phase_degrees(double new_angle, double previous_angle)
{
    while (new_angle - previous_angle > 180.0)
    {
        new_angle -= 360.0;
    }
    while (new_angle - previous_angle < -180.0)
    {
        new_angle += 360.0;
    }
    return new_angle;
}

/** \brief Reads the position.txt file from the given location, and 
 * runs the floor tracking algorithm against that data.
 */
int arwain::rerun_floor_tracker(const std::string& data_location)
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

    return arwain::ExitCodes::Success;
}

/** \brief Reads the acce.txt and gyro.txt files from the given location, 
 * and reruns the selected orientation filter(s) against that data.
 */
int arwain::rerun_orientation_filter(const std::string& data_location)
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
            slow_yaw.push_back(slow_filter.getYaw());
        }
        else
        {
            slow_yaw.push_back(unwrap_phase_degrees(slow_filter.getYaw(), slow_yaw.back()));
        }

        if (fast_yaw.empty())
        {
            fast_yaw.push_back(fast_filter.getYaw());
        }
        else
        {
            fast_yaw.push_back(unwrap_phase_degrees(fast_filter.getYaw(), fast_yaw.back()));
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

    return arwain::ExitCodes::Success;
}

void arwain::setup_log_directory()
{
    if (arwain::folder_date_string_suffix == "nullname")
    {
        arwain::folder_date_string_suffix = "";
    }
    if (arwain::folder_date_string_suffix != "")
    {
        arwain::folder_date_string = "./data_" + arwain::datetimestring() + "_" + arwain::folder_date_string_suffix;
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

    // Close the current error log, if there is one, and start a new one.
    if (arwain::error_log.is_open())
    {
        arwain::error_log.close();
    }
    arwain::error_log.open(arwain::folder_date_string + "/ERRORS.txt");
    arwain::error_log << "time event" << "\n";
}

void arwain::setup(const InputParser& input)
{
    if (input.contains("-name"))
    {
        arwain::folder_date_string_suffix = input.getCmdOption(("-name"));
    }
    else
    {
        arwain::folder_date_string_suffix = "";
    }
}

#if USE_REALSENSE == 1
void rs2_reader()
{
    // Quit immediately if disabled by config.
    if (!arwain::config.use_rs2)
    {
        return;
    }
    
    T265 t265;


    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::DataCollection:
            {
                arwain::Logger log;
                log.open(arwain::folder_date_string + "/position_t265.txt");
                log << "time x y z\n";

                while (arwain::system_mode == arwain::OperatingMode::DataCollection)
                {
                    Vector3 pos = t265.get_position();
                    log << std::chrono::system_clock::now().time_since_epoch().count() << " "
                        << pos.x << " "
                        << pos.y << " "
                        << pos.z << "\n";
                }
                break;
            }
            default:
            {
                sleep_ms(10);
                break;
            }
        }
    }
}
#endif

#if USE_UUBLA == 1
void uubla_fn()
{
    UUBLA::Network uubla{
        arwain::config.uubla_serial_port,
        arwain::config.uubla_baud_rate
    };
    arwain::uubla_handle = &uubla;

    uubla.configure("force_z_zero", true);
    uubla.configure("ewma_gain", 0.1);
    uubla.add_node_callback = inform_new_uubla_node;
    uubla.remove_node_callback = inform_remove_uubla_node;
    uubla.start_reading(); // Start as in start reading the serial port. TODO investigate this function for possible leaks.
    
    std::thread solver_th{solver_fn, &uubla};

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        sleep_ms(100);
    }

    uubla.join();
    solver_th.join();
}
#endif

int arwain::execute_inference()
{
    // Start worker threads.
    // ArwainThread imu_reader_thread(imu_reader, "arwain_imu_th");                   // Reading IMU data, updating orientation filters.
    ImuProcessing::init();
    PositionVelocityInference::init();

    // ArwainThread predict_velocity_thread(predict_velocity, "arwain_vel_th");       // Velocity and position inference.
    ArwainThread stance_detector_thread(stance_detector, "arwain_stnc_th");        // Stance, freefall, entanglement detection.
    ArwainThread transmit_lora_thread(transmit_lora, "arwain_lora_th");            // LoRa packet transmissions.
    ArwainThread std_output_thread(std_output, "arwain_std_th");                   // Prints useful output to std out.
    ArwainThread indoor_positioning_thread(indoor_positioning, "arwain_ips_th");   // Floor, stair, corner snapping.
    ArwainThread altimeter_thread(altimeter, "arwain_alt_th");                     // Uses the BMP384 sensor to determine altitude.
    #if USENCS2
        ArwainThread py_inference_thread{py_inference, "arwain_ncs2_th"};          // Temporary: Run Python script to handle velocity inference.
    #endif
    #if USE_UUBLA == 1
        std::thread uubla_thread;
        if (arwain::config.node_id == 2)
        {
            uubla_thread = std::thread{uubla_fn};
        }
    #endif
    #if USE_REALSENSE == 1
    ArwainThread rs2_thread{rs2_reader, "arwain_rs2_th"};
    #endif
    ArwainThread command_line_thread{command_line, "arwain_cmd_th"};                // Simple command line interface for runtime mode switching.

    // Wait for all threads to terminate.
    // imu_reader_thread.join();
    ImuProcessing::join();

    // predict_velocity_thread.join();
    PositionVelocityInference::join();
    
    stance_detector_thread.join();
    transmit_lora_thread.join();
    std_output_thread.join();
    indoor_positioning_thread.join();
    #if USENCS2
    py_inference_thread.join();
    #endif
    altimeter_thread.join();
    #if USE_REALSENSE == 1
    rs2_thread.join();
    #endif
    #if USE_UUBLA == 1
    if (arwain::config.node_id == 2)
    {
        uubla_thread.join();
    }
    #endif
    command_line_thread.join();

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

std::tuple<std::vector<double>, std::vector<std::vector<double>>> magcal(const std::vector<Vector3>& input_data);

int arwain::calibrate_magnetometers()
{
    LIS3MDL magnetometer{arwain::config.magn_address, arwain::config.magn_bus};
    arwain::MagnetometerCalibrator clbr;
    std::cout << "About to start magnetometer calibration." << std::endl;
    std::cout << "Move the device through all orientations; press Ctrl+C when done." << std::endl;
    sleep_ms(3000);
    std::cout << "Calibration started ..." << std::endl;

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        clbr.feed(magnetometer.read());
        sleep_ms(100);
    }
    auto [bias, scale] = clbr.solve();

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
    return arwain::ExitCodes::Success;
}

int arwain::calibrate_gyroscopes()
{
    Vector3 results;

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

std::tuple<Vector3, Vector3> deduce_calib_params(const std::vector<Vector3>& readings)
{
    double x_min = 1e6;
    double x_max = -1e6;
    double y_min = 1e6;
    double y_max = -1e6;
    double z_min = 1e6;
    double z_max = -1e6;
    
    for (auto& vec : readings)
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

int arwain::calibrate_accelerometers_simple()
{
    std::cout << "Calibrating the following accelerometers:" << std::endl;
    std::cout << "\t" << arwain::config.imu1_bus << " at 0x" << std::hex << arwain::config.imu1_address << std::endl;
    std::cout << "\t" << arwain::config.imu2_bus << " at 0x" << std::hex << arwain::config.imu2_address << std::endl;
    std::cout << "\t" << arwain::config.imu3_bus << " at 0x" << std::hex << arwain::config.imu3_address << std::endl;
    std::cout << std::dec << std::endl;

    IMU_IIM42652 imu1{arwain::config.imu1_address, arwain::config.imu1_bus};
    IMU_IIM42652 imu2{arwain::config.imu2_address, arwain::config.imu2_bus};
    IMU_IIM42652 imu3{arwain::config.imu3_address, arwain::config.imu3_bus};

    std::vector<Vector3> readings_1;
    std::vector<Vector3> readings_2;
    std::vector<Vector3> readings_3;

    // Take readings while tumbling device.
    for (int i = 0; i < 6; i++)
    {
        std::cout << i+1 << ") Place the device in a random orientation ..." << std::endl;
        sleep_ms(10000);

        Vector3 reading_1 = imu1.calibration_accel_sample();
        Vector3 reading_2 = imu2.calibration_accel_sample();
        Vector3 reading_3 = imu3.calibration_accel_sample();

        readings_1.push_back(reading_1);
        readings_2.push_back(reading_2);
        readings_3.push_back(reading_3);
    }
    
    // TODO Detect and remove outliers.
    

    // Compute centre offsets; this assumes outliers have been successfully removed.
    auto [bias_1, scale_1] = deduce_calib_params(readings_1);
    auto [bias_2, scale_2] = deduce_calib_params(readings_2);
    auto [bias_3, scale_3] = deduce_calib_params(readings_3);

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

    std::cout << "Calibration complete and logged to config file" << std::endl;

    return arwain::ExitCodes::Success;
}

RollingAverage::RollingAverage(unsigned int window_size_) : window_size(window_size_)
{
}
bool RollingAverage::ready()
{
    return stack.size() == window_size;
}
void RollingAverage::feed(double value)
{
    current_average += value;
    stack.push_back(value);
    if (stack.size() > window_size)
    {
        current_average -= stack.front() ;
        stack.pop_front();
    }
}
double RollingAverage::get_value()
{
    return current_average / static_cast<double>(window_size);
}

ActivityMetric::ActivityMetric(unsigned int ag_window_size_, unsigned int velo_window_size_)
: ag_window_size(ag_window_size_), velo_window_size(velo_window_size_)
{
    acce_roller = new RollingAverage{ag_window_size_};
    gyro_roller = new RollingAverage{ag_window_size_};
    velo_roller = new RollingAverage{velo_window_size_};
}

ActivityMetric::~ActivityMetric()
{
    delete acce_roller;
    delete gyro_roller;
    delete velo_roller;
}

void ActivityMetric::feed_gyro(const Vector3& gyro)
{
    gyro_roller->feed(gyro.magnitude());
}

void ActivityMetric::feed_acce(const Vector3& acce)
{
    acce_roller->feed(acce.magnitude());
}

void ActivityMetric::feed_velo(const Vector3& velo)
{
    velo_roller->feed(velo.magnitude());
}

double ActivityMetric::read() const
{
    return 4 * std::abs<double>(
          ((acce_roller->get_value() - acce_mean) / acce_stdv)
        * ((gyro_roller->get_value() - gyro_mean) / gyro_stdv)
        / ((velo_roller->get_value() - velo_mean) / velo_stdv)
    );
}
