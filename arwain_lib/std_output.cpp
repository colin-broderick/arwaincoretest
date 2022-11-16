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
#include "std_output.hpp"

DebugPrints::DebugPrints()
{
    init();
}

void DebugPrints::run()
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

void DebugPrints::setup_inference()
{

}

void DebugPrints::cleanup_inference()
{

}

void DebugPrints::run_inference()
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
        ss << "Stance flag:       " << arwain::stance_detection_handle->get_stance() << "\n";
        ss << "Horizontal:        " << arwain::stance_detection_handle->get_attitude() << "\n";
        ss << "Fall flag:         " << arwain::stance_detection_handle->get_falling_state() << "\n";
        ss << "Entangled flag:    " << arwain::stance_detection_handle->get_entangled_state() << "\n";

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

void DebugPrints::run_idle()
{
    sleep_ms(10);
}

void DebugPrints::core_setup()
{
}
    
bool DebugPrints::init()
{
    if (!arwain::config.log_to_stdout)
    {
        return false;
    }
    core_setup();
    job_thread = ArwainThread{&DebugPrints::run, "arwain_cout_th", this};
    return true;
}

void DebugPrints::join()
{
    if (job_thread.joinable())
    {
        job_thread.join();
    }
    std::cout << "Successfully quit DebugPrints\n";
}
