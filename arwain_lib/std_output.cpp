#include <sstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <deque>

#include "arwain.hpp"
#include "vector3.hpp"
#include "quaternion.hpp"
#include "arwain_thread.hpp"
#include "filter.hpp"
#include "exceptions.hpp"

/** \brief Periodically logs status messages to stdout.
 * Useful for debugging or testing, but probably not wanted at runtime.
 * Output is of a form that can be easily piped to other processes.
 */
namespace DebugPrints
{
    namespace // private
    {
        void run();
        void run_inference();
        void run_idle();

        ArwainThread job_thread;

        void run()
        {
            while (arwain::system_mode != arwain::OperatingMode::Terminate)
            {
                switch (arwain::system_mode)
                {
                    case arwain::OperatingMode::Inference:
                        run_inference();
                        break;
                    default:
                        run_idle();
                        break;
                }
            }
        }

        void setup_inference()
        {

        }

        void cleanup_inference()
        {

        }

        void run_inference()
        {
            setup_inference();

            // Set up timing, including pause while IMU warms up.
            std::chrono::milliseconds interval{arwain::Intervals::STD_OUT_INTERVAL};
            std::this_thread::sleep_for(interval*3);
            auto time = std::chrono::system_clock::now();

            while (arwain::system_mode == arwain::OperatingMode::Inference)
            {
                std::stringstream ss;

                ss << "Position:    " << arwain::Buffers::POSITION_BUFFER.back() << "\n";
                // Add Euler and Quaternion orientations to the string stream.
                Quaternion quat = arwain::Buffers::QUAT_ORIENTATION_BUFFER.back();
                auto euler_angles = arwain::Filter::getEulerAnglesDegrees(quat.w, quat.x, quat.y, quat.z);
                ss << "Orientation (E):   " << "R:" << euler_angles[0] << ", " << "P:" << euler_angles[1] << ", " << "Y:" << euler_angles[2] << "\n";;
                ss << "Orientation (Q):   " << quat << "\n";

                // Add stance to the string stream.
                ss << "Stance flag:       " << StanceDetection::get_stance() << "\n";
                ss << "Horizontal:        " << StanceDetection::get_attitude() << "\n";
                ss << "Fall flag:         " << StanceDetection::get_falling_state() << "\n";
                ss << "Entangled flag:    " << StanceDetection::get_entangled_state() << "\n";

                ss << "Magnetic ori (Q):  " << arwain::Buffers::MAG_ORIENTATION_BUFFER.back() << "\n";

                if (!arwain::config.no_pressure)
                {
                    ss << "Air pressure:    " << arwain::Buffers::PRESSURE_BUFFER.back() << "\n";
                }

                // Print the string.
                std::cout << ss.str() << std::endl;

                // Clear the stringstream.
                ss.str("");

                // Wait until next tick.
                time = time + interval;
                std::this_thread::sleep_until(time);
            }

            cleanup_inference();
        }

        void run_idle()
        {
            sleep_ms(10);
        }

        void core_setup()
        {
        }
    }

    // Public
    
    bool init()
    {
        if (!arwain::config.log_to_stdout)
        {
            return false;
        }
        core_setup();
        job_thread = ArwainThread{DebugPrints::run, "arwain_cout_th"};
        return true;
    }

    void join()
    {
        if (job_thread.joinable())
        {
            job_thread.join();
        }
        std::cout << "Successfully quit DebugPrints\n";
    }
}
