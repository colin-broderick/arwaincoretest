#include <sstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <deque>

#include <arwain/quaternion.hpp>
#include <arwain/vector3.hpp>
#include <arwain/timers.hpp>
#include <arwain/orientation/filter.hpp>

#include "arwain/arwain.hpp"
#include "arwain/thread.hpp"
#include "arwain/exceptions.hpp"
#include "arwain/std_output.hpp"

DebugPrints::DebugPrints()
{
    init();
}

void DebugPrints::run()
{
    if (!arwain::config.log_to_stdout)
    {
        return;
    }
    
    while (mode != arwain::OperatingMode::Terminate)
    {
        switch (mode)
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

bool DebugPrints::set_stance_detection_pointer(StanceDetection& stance)
{
    // TODO Should I delete the pointer first?
    this->stance_detection_handle = &stance;
    return true;
}

void DebugPrints::run_inference()
{
    setup_inference();

    // Set up timing, including pause while IMU warms up.
    Timers::IntervalTimer<std::chrono::milliseconds> loop_scheduler{arwain::Intervals::STD_OUT_INTERVAL, "arwain_cout_run_infer"};

    while (mode == arwain::OperatingMode::Inference)
    {
        std::stringstream ss;

        ss << "Position:    " << arwain::Buffers::POSITION_BUFFER.back() << "\n";
        // Add Euler and Quaternion orientations to the string stream.
        Quaternion quat = arwain::Buffers::QUAT_ORIENTATION_BUFFER.back();
        auto euler_angles = arwain::OrientationFilter::get_euler_angles_degrees(quat.w, quat.x, quat.y, quat.z);
        ss << "Orientation (E):   " << "R:" << euler_angles[0] << ", " << "P:" << euler_angles[1] << ", " << "Y:" << euler_angles[2] << "\n";;
        ss << "Orientation (Q):   " << quat << "\n";

        // Add stance to the string stream.
        if (stance_detection_handle != nullptr)
        {
            ss << "Stance flag:       " << this->stance_detection_handle->get_stance() << "\n";
            ss << "Horizontal:        " << this->stance_detection_handle->get_attitude() << "\n";
            ss << "Fall flag:         " << this->stance_detection_handle->get_falling_state() << "\n";
            ss << "Entangled flag:    " << this->stance_detection_handle->get_entangled_state() << "\n";
        }

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
        loop_scheduler.await();
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
    core_setup();
    job_thread = ArwainThread{&DebugPrints::run, "arwain_cout_th", this};
    return true;
}

bool DebugPrints::join()
{
    while (!job_thread.joinable())
    {
        sleep_ms(1);
    }
    if (job_thread.joinable())
    {
        job_thread.join();
        return true;
    }
    return false;
}
