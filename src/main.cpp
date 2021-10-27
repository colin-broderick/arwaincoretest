/*

Data integrity
===============================================================================

These rules are established to ensure that time-sensitive operations can execute
successfully and on time, and that data is never inadvertently changed. Deviation
from these rules should be accompanied by a comment clearly indiciating why.

- Anything accessed by more than one thread should be protected by a mutex.
- Mutex locks should be established only for the duration of a read or write and
  then released immediately. No processing should be done while data is locked.
- Any non-primitive data types should be passed by const reference unless a copy
  is explicitly required.

*/

#include <csignal>
#include <iostream>
#include <chrono>
#include <string>
#include <deque>
#include <vector>
#include <thread>
#include <mutex>
#include <fstream>
#include <experimental/filesystem>

#include "quaternion.hpp"
#include "stance.hpp"
#include "input_parser.hpp"
#include "indoor_positioning_wrapper.hpp"
#include "logger.hpp"
#include "kalman.hpp"
#include "altimeter.hpp"
#include "imu_reader.hpp"
#include "std_output.hpp"
#include "transmit_lora.hpp"
#include "velocity_prediction.hpp"
#include "utils.hpp"

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

/** \brief Capture the SIGINT signal for clean exit.
 * Sets the global SHUTDOWN flag informing all threads to clean up and exit.
 * \param signal The signal to capture.
 */
static void sigint_handler(int signal)
{
    if (signal == SIGINT)
    {
        std::cout << "\nReceived SIGINT - closing\n" << "\n";
        arwain::shutdown = 1;
    }
}

void calibrate_gyroscope_online()
{
    IMU_IIM42652 imu{0x68, "/dev/i2c-4"};

    std::cout << "Gyroscope calibration is about to start; makes sure the device is completely stationary" << std::endl;
    for (int i = 1; i < 6; i++)
    {
        std::cout << "Starting in " << 6-i << " seconds" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds{1});
    }

    std::cout << "Gyroscope calibration: Leave the device completely still for 10 seconds\n";
    int gyro_reading_count = 0;
    double gx = 0, gy = 0, gz = 0;
    for (int i = 0; i < 10*100; i++)
    {
        imu.read_IMU();
        gx += imu.gyroscope_x;
        gy += imu.gyroscope_y;
        gz += imu.gyroscope_z;
        gyro_reading_count += 1;
        if (i % 100 == 0)
        {
            std::cout << "T = " << (int)(i/100) << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    double gx_bias = gx/gyro_reading_count;
    double gy_bias = gy/gyro_reading_count;
    double gz_bias = gz/gyro_reading_count;
    arwain::config.replace("gyro_bias_x", gx_bias);
    arwain::config.replace("gyro_bias_y", gy_bias);
    arwain::config.replace("gyro_bias_z", gz_bias);

    std::cout << "Gyroscope calibration is complete, you may now operate the device normally" << std::endl;
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

/** \brief Program entry point..
 * \param argc Nmber of arguments.
 * \param argv List of arguments.
 */
int main(int argc, char **argv)
{
    // Prepare keyboard interrupt signal handler to enable graceful exit.
    std::signal(SIGINT, sigint_handler);

    // Determine behaviour from command line arguments.
    InputParser input{argc, argv};

    // Output help text.
    if (input.contains("-h") || input.contains("-help"))
    {
        std::cout << arwain::help_text << std::endl;
        return arwain::ExitCodes::Success;
    }

    // Attempt to read the config file and quit if failed.
    arwain::config = arwain::Configuration{input};
    if (!arwain::config.file_read_ok)
    {
        std::cout << "Problem reading configuration file\n";
        return arwain::ExitCodes::FailedConfiguration;
    }
    
    // Start IMU test mode. This returns so the program will quit when the test is stopped.
    if (input.contains("-testimu"))
    {
        arwain::test_imu();
        return arwain::ExitCodes::Success;
    }

    // Attempt to calibrate the gyroscope before commencing other activities.
    if (input.contains("-calib"))
    {
        calibrate_gyroscope_online();
    }

    // Create output directory and write copy of current configuration.
    if (arwain::config.log_to_file)
    {
        arwain::folder_date_string = "./data_" + arwain::datetimestring();
        if (!std::experimental::filesystem::is_directory(arwain::folder_date_string))
        {
            std::experimental::filesystem::create_directory(arwain::folder_date_string);
        }
        std::experimental::filesystem::copy(arwain::config.config_file, arwain::folder_date_string + "/config.conf");
    }

    // Open error log file (globally accessible)
    if (arwain::config.log_to_file)
    {
        arwain::error_log.open(arwain::folder_date_string + "/ERRORS.txt");
        arwain::error_log << "# time event" << "\n";
    }

    // Start worker threads.
    std::thread imu_reader_thread(imu_reader);                   // Reading IMU data, updating orientation filters.
    std::thread predict_velocity_thread(predict_velocity);       // Velocity and position inference.
    std::thread stance_detector_thread(stance_detector);         // Stance, freefall, entanglement detection.
    std::thread transmit_lora_thread(transmit_lora);             // LoRa packet transmissions.
    std::thread std_output_thread(std_output);                   // Prints useful output to std out.
    std::thread indoor_positioning_thread(indoor_positioning);   // Floor, stair, corner snapping.
    std::thread altimeter_thread(altimeter);                     // Uses the BMP280 sensor to determine altitude.
    std::thread py_inference_thread{py_inference};               // Temporary: Run Python script to handle velocity inference.
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
    // py_transmitter_thread.join();
    // kalman_filter.join();

    return arwain::ExitCodes::Success;
}
