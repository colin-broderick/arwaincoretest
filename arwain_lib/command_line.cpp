#include <map>
#include <string_view>

#include "arwain.hpp"
#include "exceptions.hpp"
#include "imu_reader.hpp"
#include "transmit_lora.hpp"
#include "altimeter.hpp"
#include "std_output.hpp"
#include "velocity_prediction.hpp"
#include "arwain_thread.hpp"
#include "indoor_positioning_wrapper.hpp"

namespace ArwainCLI
{
    std::tuple<bool, std::string> set_mode(arwain::OperatingMode new_mode);

    namespace // private
    {
        void report_current_mode();
        void fail_to_switch_to(arwain::OperatingMode mode);
        void switch_to_exit_mode();
        void switch_to_inference_mode();
        void force_switch_to_idle_autocal_mode();
        void switch_to_idle_autocal_mode();
        void switch_to_gyro_calib_mode();
        void switch_to_mag_calib_mode();
        void switch_to_accel_calib_mode();
        void switch_to_data_collection_mode();
        void set_folder_name(const std::string& input);
        void set_position_zero();
        void parse_cli_input(const std::string& input);
        
        ArwainThread job_thread;
        arwain::OperatingMode mode = arwain::OperatingMode::AutoCalibration;

        void run()
        {
            while (mode != arwain::OperatingMode::Terminate)
            {
                std::string input;
                std::getline(std::cin, input);
                parse_cli_input(input);
            }
        }

        constexpr int s2i(std::string_view input)
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
        void report_current_mode()
        {
            std::cout << "Current mode: " << arwain::system_mode << "\n";
        }

        /** \brief Report inability to switch modes and inform of current mode. */
        void fail_to_switch_to(arwain::OperatingMode mode)
        {
            std::cout << "Cannot switch to " << mode << " from current mode\n";
            report_current_mode();
        }

        /** \brief Switches the system to Terminate mode, which instructs all threads to clean up and exit.
         * This mode can be entered from any other mode.
         */
        void switch_to_exit_mode()
        {
            std::cout << "Cleaning up before closing, please wait ..." << std::endl;
            ImuProcessing::set_mode(arwain::OperatingMode::Terminate);
            PositionVelocityInference::set_mode(arwain::OperatingMode::Terminate);
            StanceDetection::set_mode(arwain::OperatingMode::Terminate);
            StatusReporting::set_mode(arwain::OperatingMode::Terminate);
            DebugPrints::set_mode(arwain::OperatingMode::Terminate);
            Altimeter::set_mode(arwain::OperatingMode::Terminate);
            ArwainCLI::set_mode(arwain::OperatingMode::Terminate);
            IndoorPositioningSystem::set_mode(arwain::OperatingMode::Terminate);
            arwain::system_mode = arwain::OperatingMode::Terminate;
        }

        /** \brief Switches the system to Inference mode. Can only be entered from Idle/Autocalibration mode. */
        void switch_to_inference_mode()
        {
            if (arwain::system_mode == arwain::OperatingMode::Inference)
            {
                std::cout << "Already in inference mode" << std::endl;
            }
            else if (!arwain::ready_for_inference)
            {
                std::cout << "Not yet ready for inference; wait a few seconds and try again ..." << std::endl;
            }
            else if (arwain::system_mode == arwain::OperatingMode::AutoCalibration)
            {
                std::cout << "Entering inference mode" << std::endl;
                arwain::setup_log_directory();
                ImuProcessing::set_mode(arwain::OperatingMode::Inference);
                PositionVelocityInference::set_mode(arwain::OperatingMode::Inference);
                StanceDetection::set_mode(arwain::OperatingMode::Inference);
                StatusReporting::set_mode(arwain::OperatingMode::Inference);
                DebugPrints::set_mode(arwain::OperatingMode::Inference);
                Altimeter::set_mode(arwain::OperatingMode::Inference);
                ArwainCLI::set_mode(arwain::OperatingMode::Inference);
                IndoorPositioningSystem::set_mode(arwain::OperatingMode::Inference);
                arwain::system_mode = arwain::OperatingMode::Inference;
            }
            else
            {
                std::cout << "Cannot switch to inference from current mode\n";
                report_current_mode();
            }
        }

        void force_switch_to_idle_autocal_mode()
        {
            arwain::system_mode = arwain::OperatingMode::AutoCalibration;
        }

        /** \brief Puts the system into Idle mode. Only reachable from Inference mode.
         * Other modes will put the system into this Idle mode automatically when their
         * work is complete.
         */
        void switch_to_idle_autocal_mode()
        {
            if (arwain::system_mode == arwain::OperatingMode::Inference)
            {
                std::cout << "Entering autocalibration mode" << std::endl;
                arwain::system_mode = arwain::OperatingMode::AutoCalibration;
                ImuProcessing::set_mode(arwain::OperatingMode::AutoCalibration);
                PositionVelocityInference::set_mode(arwain::OperatingMode::AutoCalibration);
                StanceDetection::set_mode(arwain::OperatingMode::AutoCalibration);
                StatusReporting::set_mode(arwain::OperatingMode::AutoCalibration);
                DebugPrints::set_mode(arwain::OperatingMode::AutoCalibration);
                Altimeter::set_mode(arwain::OperatingMode::AutoCalibration);
                ArwainCLI::set_mode(arwain::OperatingMode::AutoCalibration);
                IndoorPositioningSystem::set_mode(arwain::OperatingMode::AutoCalibration);
            }
            else
            {
                fail_to_switch_to(arwain::OperatingMode::AutoCalibration);
            }
        }

        /** \brief Switch to active gyro calibration mode. Only reachable from Idle/Autocal modes. */
        void switch_to_gyro_calib_mode()
        {
            if (arwain::system_mode != arwain::OperatingMode::AutoCalibration)
            {
                fail_to_switch_to(arwain::OperatingMode::GyroscopeCalibration);
            }
            else
            {
                std::cout << "Starting gyroscope calibration" << std::endl;
                arwain::system_mode = arwain::OperatingMode::GyroscopeCalibration;
                PositionVelocityInference::set_mode(arwain::OperatingMode::GyroscopeCalibration);
                StanceDetection::set_mode(arwain::OperatingMode::GyroscopeCalibration);
                StatusReporting::set_mode(arwain::OperatingMode::GyroscopeCalibration);
                DebugPrints::set_mode(arwain::OperatingMode::GyroscopeCalibration);
                Altimeter::set_mode(arwain::OperatingMode::GyroscopeCalibration);
                ArwainCLI::set_mode(arwain::OperatingMode::GyroscopeCalibration);
                IndoorPositioningSystem::set_mode(arwain::OperatingMode::GyroscopeCalibration);
                ImuProcessing::set_mode(arwain::OperatingMode::GyroscopeCalibration, std::function<void()>(force_switch_to_idle_autocal_mode));
            }
        }

        /** \brief Switch to active magnetometer calibration mode. Only reachable from Idle/Autocal modes. */
        void switch_to_mag_calib_mode()
        {
            if (arwain::system_mode != arwain::OperatingMode::AutoCalibration)
            {
                fail_to_switch_to(arwain::OperatingMode::MagnetometerCalibration);
            }
            else
            {
                std::cout << "Starting magnetometer calibration" << std::endl;
                ImuProcessing::set_mode(arwain::OperatingMode::MagnetometerCalibration);
                PositionVelocityInference::set_mode(arwain::OperatingMode::MagnetometerCalibration);
                StanceDetection::set_mode(arwain::OperatingMode::MagnetometerCalibration);
                StatusReporting::set_mode(arwain::OperatingMode::MagnetometerCalibration);
                DebugPrints::set_mode(arwain::OperatingMode::MagnetometerCalibration);
                Altimeter::set_mode(arwain::OperatingMode::MagnetometerCalibration);
                ArwainCLI::set_mode(arwain::OperatingMode::MagnetometerCalibration);
                IndoorPositioningSystem::set_mode(arwain::OperatingMode::MagnetometerCalibration);
                arwain::system_mode = arwain::OperatingMode::MagnetometerCalibration;
            }
        }

        /** \brief Switch to active accelerometer calibration mode. Only reachable from Idle/Autocal modes. */
        void switch_to_accel_calib_mode()
        {
            if (arwain::system_mode != arwain::OperatingMode::AutoCalibration)
            {
                fail_to_switch_to(arwain::OperatingMode::AccelerometerCalibration);
            }
            else
            {
                std::cout << "Starting accelerometer calibration" << std::endl;
                ImuProcessing::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                PositionVelocityInference::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                StanceDetection::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                StatusReporting::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                DebugPrints::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                Altimeter::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                ArwainCLI::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                IndoorPositioningSystem::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                arwain::system_mode = arwain::OperatingMode::AccelerometerCalibration;
            }
        }

        void switch_to_data_collection_mode()
        {
            if (arwain::system_mode != arwain::OperatingMode::AutoCalibration)
            {
                fail_to_switch_to(arwain::OperatingMode::DataCollection);
            }
            else
            {
                // TODO What's going on here?
                throw NotImplemented{"SEARCHTHISSTRING00000123"};
                std::cout << "Starting accelerometer calibration" << std::endl;
                ImuProcessing::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                PositionVelocityInference::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                StanceDetection::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                StatusReporting::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                DebugPrints::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                Altimeter::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                ArwainCLI::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                IndoorPositioningSystem::set_mode(arwain::OperatingMode::AccelerometerCalibration);
                arwain::system_mode = arwain::OperatingMode::AccelerometerCalibration;
            }
        }

        void set_folder_name(const std::string& input)
        {
            if (arwain::system_mode != arwain::OperatingMode::AutoCalibration)
            {
                std::cout << "Folder name can only be changed from idle/autocalibration modes" << std::endl;
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

        /** \brief Sets the most recent entry in the world position buffer equal to (0, 0, 0). A flag is raised
         * which causes the velocity inference loop to set the position to zero at the next iteration of the loop.
        */
        void set_position_zero()
        {
            arwain::reset_position = true;
        }

        void parse_cli_input(const std::string& input)
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
                case s2i("zeropos"):
                    set_position_zero();
                    break;
                default:
                    std::cout << "ERROR: Command not recognised: " << input << std::endl;
                    break;
            }
        }

        void core_setup()
        {
        }
    }

    // Public

    bool init()
    {
        core_setup();
        job_thread = ArwainThread{ArwainCLI::run, "arwain_cmdl_th"};
        return true;
    }

    bool shutdown()
    {
        mode = arwain::OperatingMode::Terminate;
        return true;
    }

    void join()
    {
        job_thread.join();
    }

    std::tuple<bool, std::string> set_mode(arwain::OperatingMode new_mode)
    {
        mode = new_mode;
        return {true, "success"};
    }
}
