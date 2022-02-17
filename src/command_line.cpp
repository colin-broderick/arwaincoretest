#include "arwain.hpp"

void command_line()
{
    // TODO Take care to update any global state when switching from or to any relevant modes.
    // An important example is that the folder_date_string should change when starting a new logging session,
    // i.e. when switching to inference mode. Different logging might become relevant in other modes, so that
    // might be important there too.

    // TODO APIs for request of runtime status? I added request for gyro calib simply to confirm it was auto
    // calibrating as expected, but it's not a good implementation.

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        std::string input;
        std::cout << "ARWAIN > ";
        std::getline(std::cin, input);

        if (input == "quit" || input == "exit" || input == "shutdown" || input == "stop")
        {
            std::cout << "Cleaning up before closing, please wait ..." << std::endl;
            arwain::system_mode = arwain::OperatingMode::Terminate;
        }
        else if (input == "inference" || input == "infer")
        {
            if (arwain::system_mode == arwain::OperatingMode::Inference)
            {
                std::cout << "Already in inference mode" << std::endl;
            }
            else if (!arwain::ready_for_inference)
            {
                std::cout << "Not yet ready for inference; wait a few seconds and try again ..." << std::endl;
            }
            else
            {
                std::cout << "Entering inference mode" << std::endl;
                arwain::setup_log_directory();
                arwain::system_mode = arwain::OperatingMode::Inference;
            }
        }
        else if (input == "infer10")
        {
            if (arwain::system_mode == arwain::OperatingMode::Inference)
            {
                std::cout << "Already in inference mode" << std::endl;
            }
            else if (!arwain::ready_for_inference)
            {
                std::cout << "Not yet ready for inference; wait a few seconds and try again ..." << std::endl;
            }
            else
            {
                std::cout << "Entering inference mode in 10 seconds ..." << std::endl;
                sleep_ms(10000);
                std::cout << "Inference started" << std::endl;
                arwain::setup_log_directory();
                arwain::system_mode = arwain::OperatingMode::Inference;
            }
        }
        else if (input == "gyro")
        {
            arwain::request_gyro_calib = true;
            sleep_ms(10);
        }
        else if (input == "autocal" || input == "idle")
        {
            std::cout << "Entering autocalibration mode" << std::endl;
            arwain::system_mode = arwain::OperatingMode::AutoCalibration;
        }
        else if (input == "mode")
        {
            std::cout << "Current mode: " << arwain::system_mode << std::endl;
        }
        else if (input == "calibg")
        {
            if (arwain::system_mode != arwain::OperatingMode::AutoCalibration)
            {
                std::cout << "error: Active calibration modes can only be entered from idle/autocalibration modes" << std::endl;
            }
            else
            {
                std::cout << "Starting gyroscope calibration" << std::endl;
                arwain::system_mode = arwain::OperatingMode::GyroscopeCalibration;
            }
        }
        else if (input == "testuwb")
        {
            arwain::system_mode = arwain::OperatingMode::TestSerial;
        }
        else if (input == "teststance")
        {
            // TODO Function to check for validity of mode transitions.
            arwain::system_mode = arwain::OperatingMode::TestStanceDetector;
        }
        else if (input == "calibm")
        {
            if (arwain::system_mode != arwain::OperatingMode::AutoCalibration)
            {
                std::cout << "error: Active calibration modes can only be entered from idle/autocalibration modes" << std::endl;
            }
            else
            {
                std::cout << "Starting magnetometer calibration" << std::endl;
                arwain::system_mode = arwain::OperatingMode::MagnetometerCalibration;
            }
        }
        else if (input == "caliba")
        {
            if (arwain::system_mode != arwain::OperatingMode::AutoCalibration)
            {
                std::cout << "error: Active calibration modes can only be entered from idle/autocalibration modes" << std::endl;
            }
            else
            {
                std::cout << "Starting accelerometer calibration" << std::endl;
                arwain::system_mode = arwain::OperatingMode::AccelerometerCalibration;
            }
        }
        else if (input.substr(0, 4) == "name")
        {
            if (arwain::system_mode != arwain::OperatingMode::AutoCalibration)
            {
                std::cout << "error: Folder name can only be changed from idle/autocalibration modes" << std::endl;
            }
            else
            {
                std::string value;
                std::istringstream iss{input};
                iss >> value >> value;
                if (value == "name")
                {
                    std::cout << "error: Provide a valid name" << std::endl;
                }
                else
                {
                    arwain::folder_date_string_suffix = value;
                    std::cout << "Log folder name suffux set to '" << value << "'" << std::endl;
                }
            }
        }
        else
        {
            std::cout << "error: unrecognised command: " << input << std::endl;
        }
    }
}
