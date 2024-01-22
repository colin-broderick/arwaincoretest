#include "arwain/hybrid_positioner.hpp"

HybridPositioner::HybridPositioner()
{
    init();
}

bool HybridPositioner::init()
{
    core_setup();
    job_thread = std::jthread{std::bind_front(&HybridPositioner::run, this)};
    return true;
}

bool HybridPositioner::join()
{
    if (job_thread.joinable())
    {
        job_thread.join();
        return true;
    }
    return false; // This won't need to return a bool if it's based on jthread and autojoins - it either returns or fails, not both.
}

void HybridPositioner::run()
{
    while (mode != arwain::OperatingMode::Terminate)
    {
        switch (mode)
        {
        case arwain::OperatingMode::GyroscopeCalibration:
        case arwain::OperatingMode::MagnetometerCalibration:
        case arwain::OperatingMode::TestStanceDetector:
        case arwain::OperatingMode::AccelerometerCalibration:
        case arwain::OperatingMode::SelfTest:
        case arwain::OperatingMode::Idle:
            run_idle();
            break;
        case arwain::OperatingMode::Inference:
            run_inference();
            break;
        default:
            sleep_ms(10);
            break;
        }
    }
}

void HybridPositioner::core_setup()
{
    // Currently no core_setup to do; might come later.
}

void HybridPositioner::run_inference()
{
    setup_inference();

    Timers::IntervalTimer<std::chrono::milliseconds> loop_scheduler{100, "hybrid_position_timer"}; // TODO Consider timing.

    while (mode == arwain::OperatingMode::Inference)
    {
        // TODO Probably need to also consider orientation.
        hyb.update({}, {});
        sleep_ms(10);
    }

    cleanup_inference();
}

void HybridPositioner::run_idle()
{
    while (mode == arwain::OperatingMode::Idle)
    // TODO This loop condition is flawed since several modes can bring us here,
    // and several modes can bring us out.
    {
        sleep_ms(10);
    }
}

void HybridPositioner::setup_inference()
{
    hybrid_pos_log.open(arwain::folder_date_string + "/hyrid_position.txt");
    hybrid_pos_log << "time x y z\n"; // TODO Consider exactly what to log.
}

bool HybridPositioner::cleanup_inference()
{
    return hybrid_pos_log.close();
}

Vector3 HybridPositioner::get_position() const
{
    return hyb.get_position();
}

Vector3 HybridPositioner::PositionHybridizer::update(Vector3 inertial_position, Vector3 uwb_position)
{
    // TODO
    return (inertial_position + uwb_position) / 2.0;
}

Vector3 HybridPositioner::PositionHybridizer::get_position() const
{
    return position;
}
