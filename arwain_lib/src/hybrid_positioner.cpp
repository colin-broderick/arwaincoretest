#include "arwain/hybrid_positioner.hpp"
#include "arwain/velocity_prediction.hpp"
#include "arwain/uwb_reader.hpp"
#include "arwain/service_manager.hpp"
#include "arwain/events.hpp"

HybridPositioner::HybridPositioner()
: vel_event_id{arwain::Events::new_arwain_velocity_event.add_callback(
    std::bind(&HybridPositioner::new_inertial_velocity_callback, this, std::placeholders::_1)
    )},
  pos_event_id{arwain::Events::new_uwb_position_event.add_callback(
    std::bind(&HybridPositioner::new_uwb_position_callback, this, std::placeholders::_1)
    )}
{
    init();
}

HybridPositioner::~HybridPositioner()
{
    // TODO Should be easy enough to create an RAII type for registrations that won't need explicit deletion.
    arwain::Events::new_arwain_velocity_event.remove_callback(vel_event_id);
    arwain::Events::new_uwb_position_event.remove_callback(pos_event_id);
}

void HybridPositioner::new_inertial_velocity_callback(arwain::Events::Vector3EventWithDt inertial_velocity)
{
    hyb.position = hyb.position + inertial_velocity.velocity * inertial_velocity.dt;
}

void HybridPositioner::new_uwb_position_callback(arwain::Events::Vector3EventWithDt uwb_position)
{
    static double gain = 0;
    hyb.position = hyb.position + gain * (hyb.position - uwb_position.position);
}

bool HybridPositioner::init()
{
    core_setup();
    job_thread = std::jthread{std::bind_front(&HybridPositioner::run, this)};
    return true;
}

bool HybridPositioner::join()
{
    return false; // This won't need to return a bool if it's based on jthread and autojoins - it either returns or fails, not both.
}

void HybridPositioner::run_idle()
{
    sleep_ms(10);
}

void HybridPositioner::run()
{
    while (mode != arwain::OperatingMode::Terminate)
    {
        switch (mode)
        {
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
    // The things I do here probably seem pointless but matching the pattern
    // of the other ArwainJobs was the easiest way to get the job done. Changes will
    // need to consider a refactor of the whole ArwainJob inheritance pattern.

    setup_inference();

    Timers::IntervalTimer<std::chrono::milliseconds> loop_scheduler{100, "hybrid_position_timer"};

    while (mode == arwain::OperatingMode::Inference)
    {
        sleep_ms(10);
    }

    cleanup_inference();
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
    // TODO The concept of an update function may be unnecessary.
    return Vector3::Zero;
}

Vector3 HybridPositioner::PositionHybridizer::get_position() const
{
    return position;
}
