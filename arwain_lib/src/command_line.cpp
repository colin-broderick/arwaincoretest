#include <map>
#include <string_view>

#include "arwain/arwain.hpp"
#include "arwain/exceptions.hpp"
#include "arwain/sensor_manager.hpp"
#include "arwain/transmit_lora.hpp"
#include "arwain/altimeter.hpp"
#include "arwain/std_output.hpp"
#include "arwain/velocity_prediction.hpp"
#include "arwain/indoor_positioning_wrapper.hpp"
#include "arwain/command_line.hpp"
#include "arwain/events.hpp"

ArwainCLI::ArwainCLI()
: ArwainCLI(std::cin)
{
}

ArwainCLI::ArwainCLI(const std::istream& stream)
{
    in_stream = std::make_shared<std::istream>(stream.rdbuf());
    init();
}

void ArwainCLI::force_switch_to_idle_autocal_mode()
{
    arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Idle);
}

void ArwainCLI::run()
{
    if (arwain::config.no_cli)
    {
        return;
    }
    while (mode != arwain::OperatingMode::Terminate)
    {
        sleep_ms(5);
        std::string input;
        std::getline(*in_stream, input);
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
std::string ArwainCLI::report_current_mode() const
{
    std::stringstream ss;
    ss << "Current mode: " << mode << "\n";
    std::string s = ss.str();
    return s;
}

/** \brief Report inability to switch modes and inform of current mode. */
void ArwainCLI::fail_to_switch_to(arwain::OperatingMode mode)
{
    std::cout << "Cannot switch to " << mode << " from current " << get_mode() << "\n";
}

/** \brief Switches the system to Terminate mode, which instructs all threads to clean up and exit.
 * This mode can be entered from any other mode.
 * \return Boolean success or failure to switch mode. This one can't fail but it returns a bool for
 * API consistency.
 */
bool ArwainCLI::switch_to_exit_mode()
{
    std::cout << "Cleaning up before closing, please wait ..." << std::endl;
    arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Terminate);
    return true;
}

/** \brief Switches the system to Inference mode. Can only be entered from Idle mode. */
void ArwainCLI::switch_to_inference_mode()
{
    if (mode == arwain::OperatingMode::Inference)
    {
        std::cout << "Already in inference mode" << std::endl;
    }
    else if (mode == arwain::OperatingMode::Idle)
    {
        std::cout << "Entering inference mode" << std::endl;
        arwain::setup_log_directory();
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Inference);
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
 * \return Boolean success or failure to switch mode.
 */
bool ArwainCLI::switch_to_idle_autocal_mode()
{
    if (mode == arwain::OperatingMode::Inference)
    {
        std::cout << "Entering idle mode" << std::endl;
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::Idle);
        return true;
    }
    else
    {
        fail_to_switch_to(arwain::OperatingMode::Idle);
        return false;
    }
}

/** \brief Switch to active gyro calibration mode. Only reachable from Idle/Autocal modes.
 * \return Boolean success or failure to switch mode.
 */
bool ArwainCLI::switch_to_gyro_calib_mode()
{
    if (mode != arwain::OperatingMode::Idle)
    {
        fail_to_switch_to(arwain::OperatingMode::GyroscopeCalibration);
        return false;
    }
    else
    {
        std::cout << "Starting gyroscope calibration" << std::endl;
        // FIXTODAY ImuProcessing::set_post_gyro_calibration_callback(std::function<void()>(force_switch_to_idle_autocal_mode));
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::GyroscopeCalibration);
        return true;
    }
}

/** \brief Switch to active magnetometer calibration mode. Only reachable from Idle/Autocal modes.
 * \return Boolean success or failure to switch mode.
 */
bool ArwainCLI::switch_to_mag_calib_mode()
{
    if (mode != arwain::OperatingMode::Idle)
    {
        fail_to_switch_to(arwain::OperatingMode::MagnetometerCalibration);
        return false;
    }
    else
    {
        std::cout << "Starting magnetometer calibration" << std::endl;
        // FIXTODAY ImuProcessing::set_post_gyro_calibration_callback(std::function<void()>(force_switch_to_idle_autocal_mode));
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::MagnetometerCalibration);
        return true;
    }
}

/** \brief Switch to active accelerometer calibration mode. Only reachable from Idle/Autocal modes.
 * \return Boolean success or failure to switch mode.
 */
bool ArwainCLI::switch_to_accel_calib_mode()
{
    if (mode != arwain::OperatingMode::Idle)
    {
        fail_to_switch_to(arwain::OperatingMode::AccelerometerCalibration);
        return false;
    }
    else
    {
        std::cout << "Starting accelerometer calibration" << std::endl;
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::AccelerometerCalibration);
        return true;
    }
}

/** \brief Switch to data collection mode. Only reachable from Idle/Autocal mode.
 * \return Boolean success or failure to switch mode.
 */
bool ArwainCLI::switch_to_data_collection_mode()
{
    if (mode != arwain::OperatingMode::Idle)
    {
        fail_to_switch_to(arwain::OperatingMode::DataCollection);
        return false;
    }
    else
    {
        std::cout << "Starting data collection calibration" << std::endl;
        arwain::Events::switch_mode_event.invoke(arwain::OperatingMode::DataCollection);
        return true;
    }
}

/** \brief Set the folder name for logs from the Arwain CLI by typing "name content", where content
 * is the label to use. Will only succeed in Idle mode. Will fail with invalid input such as
 * "name" with no "content".
 * \param input A string of form "name content" where content is the name to apply.
 * \return Boolean success or failure to switch mode.
 */
bool ArwainCLI::set_folder_name(const std::string& input)
{
    if (mode != arwain::OperatingMode::Idle)
    {
        std::cout << "Folder name can only be changed from idle mode" << std::endl;
        report_current_mode();
        return false;
    }

    std::string value;
    std::istringstream iss{input};
    iss >> value >> value;

    if (value == "name")
    {
        std::cout << "ERROR: Provide a valid name" << std::endl;
        return false;
    }

    arwain::folder_date_string_suffix = value;
    std::cout << "Log folder name suffix set to '" << value << "'" << std::endl;

    return true;
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
    core_setup();
    job_thread = std::jthread{std::bind_front(&ArwainCLI::run, this)};
    return true;
}

bool ArwainCLI::join()
{
    if (job_thread.joinable())
    {
        job_thread.join();
        return true;
    }
    return false;
}

void ArwainCLI::run_inference()
{
    // To satisfy interface
}

void ArwainCLI::run_idle()
{
    // To satisfy interface
}

void ArwainCLI::setup_inference()
{
    // To satisfy interface
}

bool ArwainCLI::cleanup_inference()
{
    // To satisfy interface
    return true;
}
