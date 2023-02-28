#include <map>
#include <string_view>

#include "arwain.hpp"
#include "exceptions.hpp"
#include "sensor_manager.hpp"
#include "transmit_lora.hpp"
#include "altimeter.hpp"
#include "std_output.hpp"
#include "velocity_prediction.hpp"
//#include "arwain_thread.hpp"
#include "indoor_positioning_wrapper.hpp"
#include "command_line.hpp"

ArwainCLI::ArwainCLI()
{
    init();
}

void ArwainCLI::force_switch_to_idle_autocal_mode()
{
    arwain::system_mode = arwain::OperatingMode::Idle;
}

void ArwainCLI::run()
{
    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        std::string input;
        std::getline(std::cin, input);
        std::cout << "past get line" << std::endl;
        parse_cli_input(input);
    }
}

constexpr int ArwainCLI::s2i(std::string_view input)
{
    enum { QUIT, INFER, AUTOCAL, MODE, CALIBG, CALIBA, CALIBM, NAME, DEFAULT, DATACOLLECTION, HELP, ZEROPOS };

    if (input == "quit" || input == "stop" || input == "shutdown" || input == "exit") return QUIT;
    if (input == "infer" || input == "inference") return INFER;
    if (input == "autocal" || input == "idle") return AUTOCAL;
    if (input == "mode") return MODE;
    if (input == "calibg") return CALIBG;
    if (input == "calibm") return CALIBM;
    if (input == "caliba") return CALIBA;
    if (input == "record") return DATACOLLECTION;
    if (input == "name") return NAME;
    if (input == "help") return HELP;
    if (input == "zeropos") return ZEROPOS;
    if (input.substr(0, 4) == "name") return NAME;

    else return DEFAULT;
}

/** \brief Reports the current system mode. */
void ArwainCLI::report_current_mode()
{
    std::cout << "Current mode: " << arwain::system_mode << "\n";
}

/** \brief Report inability to switch modes and inform of current mode. */
void ArwainCLI::fail_to_switch_to(arwain::OperatingMode mode)
{
    std::cout << "Cannot switch to " << mode << " from current mode\n";
    report_current_mode();
}

/** \brief Switches the system to Terminate mode, which instructs all threads to clean up and exit.
 * This mode can be entered from any other mode.
 */
void ArwainCLI::switch_to_exit_mode()
{
    std::cout << "Cleaning up before closing, please wait ..." << std::endl;
    arwain::system_mode = arwain::OperatingMode::Terminate;
}


bool ArwainCLI::set_velocity_inference_pointer(PositionVelocityInference& velocity)
{
    // TODO Should I delete the pointer first?
    this->velocity_inference_handle = &velocity;
    return true;
}

/** \brief Switches the system to Inference mode. Can only be entered from Idle mode. */
void ArwainCLI::switch_to_inference_mode()
{
    if (arwain::system_mode == arwain::OperatingMode::Inference)
    {
        std::cout << "Already in inference mode" << std::endl;
    }
    else if (this->velocity_inference_handle->ready())
    {
        std::cout << "Not yet ready for inference; wait a few seconds and try again ..." << std::endl;
    }
    else if (arwain::system_mode == arwain::OperatingMode::Idle)
    {
        std::cout << "Entering inference mode" << std::endl;
        arwain::setup_log_directory();
        arwain::system_mode = arwain::OperatingMode::Inference;
    }
    else
    {
        std::cout << "Cannot switch to inference from current mode\n";
        report_current_mode();
    }
}


/** \brief Puts the system into Idle mode. Only reachable from Inference mode.
 * Other modes will put the system into this Idle mode automatically when their
 * work is complete.
 */
void ArwainCLI::switch_to_idle_autocal_mode()
{
    if (arwain::system_mode == arwain::OperatingMode::Inference)
    {
        std::cout << "Entering idle mode" << std::endl;
        arwain::system_mode = arwain::OperatingMode::Idle;
    }
    else
    {
        fail_to_switch_to(arwain::OperatingMode::Idle);
    }
}

/** \brief Switch to active gyro calibration mode. Only reachable from Idle/Autocal modes. */
void ArwainCLI::switch_to_gyro_calib_mode()
{
    if (arwain::system_mode != arwain::OperatingMode::Idle)
    {
        fail_to_switch_to(arwain::OperatingMode::GyroscopeCalibration);
    }
    else
    {
        std::cout << "Starting gyroscope calibration" << std::endl;
        // FIXTODAY ImuProcessing::set_post_gyro_calibration_callback(std::function<void()>(force_switch_to_idle_autocal_mode));
        arwain::system_mode = arwain::OperatingMode::GyroscopeCalibration;
    }
}

/** \brief Switch to active magnetometer calibration mode. Only reachable from Idle/Autocal modes. */
void ArwainCLI::switch_to_mag_calib_mode()
{
    if (arwain::system_mode != arwain::OperatingMode::Idle)
    {
        fail_to_switch_to(arwain::OperatingMode::MagnetometerCalibration);
    }
    else
    {
        std::cout << "Starting magnetometer calibration" << std::endl;
        // FIXTODAY ImuProcessing::set_post_gyro_calibration_callback(std::function<void()>(force_switch_to_idle_autocal_mode));
        arwain::system_mode = arwain::OperatingMode::MagnetometerCalibration;
    }
}

/** \brief Switch to active accelerometer calibration mode. Only reachable from Idle/Autocal modes. */
void ArwainCLI::switch_to_accel_calib_mode()
{
    if (arwain::system_mode != arwain::OperatingMode::Idle)
    {
        fail_to_switch_to(arwain::OperatingMode::AccelerometerCalibration);
    }
    else
    {
        std::cout << "Starting accelerometer calibration" << std::endl;
        arwain::system_mode = arwain::OperatingMode::AccelerometerCalibration;
    }
}

void ArwainCLI::switch_to_data_collection_mode()
{
    if (arwain::system_mode != arwain::OperatingMode::Idle)
    {
        fail_to_switch_to(arwain::OperatingMode::DataCollection);
    }
    else
    {
        // TODO What's going on here?
        throw NotImplemented{"SEARCHTHISSTRING00000123"};
        std::cout << "Starting accelerometer calibration" << std::endl;
        arwain::system_mode = arwain::OperatingMode::AccelerometerCalibration;
    }
}

void ArwainCLI::set_folder_name(const std::string& input)
{
    if (arwain::system_mode != arwain::OperatingMode::Idle)
    {
        std::cout << "Folder name can only be changed from idle mode" << std::endl;
        report_current_mode();
    }
    else
    {
        std::string value;
        std::istringstream iss{input};
        iss >> value >> value;
        if (value == "name")
        {
            std::cout << "ERROR: Provide a valid name" << std::endl;
        }
        else
        {
            arwain::folder_date_string_suffix = value;
            std::cout << "Log folder name suffux set to '" << value << "'" << std::endl;
        }
    }
}

void ArwainCLI::parse_cli_input(const std::string& input)
{
    switch (s2i(input.c_str()))
    {
        case s2i("quit"):
            switch_to_exit_mode();
            break;
        case s2i("infer"):
            switch_to_inference_mode();
            break;
        case s2i("autocal"):
            switch_to_idle_autocal_mode();
            break;
        case s2i("mode"):
            report_current_mode();
            break;
        case s2i("calibg"):
            switch_to_gyro_calib_mode();
            break;
        case s2i("calibm"):
            switch_to_mag_calib_mode();
            break;
        case s2i("caliba"):
            switch_to_accel_calib_mode();
            break;
        case s2i("record"):
            switch_to_data_collection_mode();
            break;
        case s2i("name"):
            set_folder_name(input);
            break;
        case s2i("help"):
            std::cout << arwain::help_text << "\n";
            break;
        default:
            std::cout << "ERROR: Command not recognised: " << input << std::endl;
            break;
    }
}

void ArwainCLI::core_setup()
{
}

bool ArwainCLI::init()
{
    if (arwain::config.no_cli)
    {
        return false;
    }
    core_setup();
    job_thread = ArwainThread{&ArwainCLI::run, "arwain_cmdl_th", this};
    return true;
}

void ArwainCLI::join()
{
    if (job_thread.joinable())
    {
        job_thread.join();
    }
}