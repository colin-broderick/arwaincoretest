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
#include <cstdlib>
#include <experimental/filesystem>

#ifdef USEROS
#include <ros/ros.h>
#endif

#include "arwain.hpp"
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
#include "timers.hpp"
#include "arwain.hpp"
#include "arwain_tests.hpp"
#include "calibration.hpp"

/** \brief Capture the SIGINT signal for clean exit.
 * Sets the system mode to Terminate, which instructs all threads to clean up and exit.
 * \param signal The signal to capture.
 */
static void sigint_handler(int signal)
{
    if (signal == SIGINT)
    {
        std::cout << "\nReceived SIGINT - closing\n" << "\n";
        arwain::system_mode = arwain::OperatingMode::Terminate;
    }
}

/** \brief Program entry point..
 * \param argc Nmber of arguments.
 * \param argv List of arguments.
 */
int main(int argc, char **argv)
{
    #ifdef USEROS
    ros::init(argc, argv, "mag_calib_source");
    #endif

    int ret;

    // Prepare keyboard interrupt signal handler to enable graceful exit.
    std::signal(SIGINT, sigint_handler);

    // Determine behaviour from command line arguments.
    InputParser input{argc, argv};

    // Output help text if requested.
    if (input.contains("-h") || input.contains("-help"))
    {
        std::cout << arwain::help_text << std::endl;
        return arwain::ExitCodes::Success;
    }

    // Attempt to read the config file and quit if failed.
    arwain::config = arwain::Configuration{input};
    if ((ret = arwain::config.read_from_file()) != arwain::Configuration::ReturnCodes::OK)
    {
        std::cout << arwain::ErrorMessages[ret] << std::endl;
    }
    
    // Start IMU test mode. This returns so the program will quit when the test is stopped.
    else if (input.contains("-testimu"))
    {
        ret = arwain::test_imu();
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
        const char *rate_str = input.getCmdOption("-testori").c_str();
        rate = std::atoi(rate_str);
        ret = arwain::test_ori(rate);
    }
    else if (input.contains("-rerunori"))
    {
        ret = arwain::rerun_orientation_filter(input.getCmdOption("-rerunori"));
    }
    else if (input.contains("-rerunfloor"))
    {
        ret = arwain::rerun_floor_tracker(input.getCmdOption("-rerunfloor"));
    }

    // Perform quick calibration of gyroscopes and write to config file.
    else if (input.contains("-calibg"))
    {
        ret = arwain::calibrate_gyroscopes();
    }

    // Perform quick calibration of gyroscopes and write to config file.
    else if (input.contains("-caliba"))
    {
        ret = arwain::calibrate_accelerometers_simple();
    }
    else if (input.contains("-testmag"))
    {
        #ifdef USEROS
        ret = arwain::test_mag(argc, argv);
        #else
        ret = arwain::test_mag();
        #endif
    }

    else
    {
        // Attempt to calibrate the gyroscope before commencing other activities.
        if (input.contains("-calib"))
        {
            arwain::calibrate_gyroscopes();
            arwain::config = arwain::Configuration{input}; // Reread the config file as it has now changed.
        }
        arwain::setup(input);
        ret = arwain::execute_inference();
    }

    return ret;
}
