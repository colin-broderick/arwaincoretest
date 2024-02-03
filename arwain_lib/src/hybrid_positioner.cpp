#include <iostream>

#include <arwain/icp.hpp>

#include "arwain/hybrid_positioner.hpp"
#include "arwain/velocity_prediction.hpp"
#include "arwain/uwb_reader.hpp"
#include "arwain/service_manager.hpp"
#include "arwain/events.hpp"




/*
Keep 10-second record of pure uwb position
Keep 10-second record of pure inertial position.
Remap both so originate at (0, 0, 0).
Do ICP to deduce best rotation of inertial onto UWB.
All new points rotated by current_angular_correction.
Current_angular_correction is updated every frame.





*/


static std::vector<Vector3> inertial_positions;
static std::vector<Vector3> uwb_positions;

HybridPositioner::HybridPositioner()
    : imu_vel_event_id{arwain::Events::new_arwain_velocity_event.add_callback(
          std::bind(&HybridPositioner::new_inertial_velocity_callback, this, std::placeholders::_1))},
      imu_pos_event_id{arwain::Events::new_arwain_velocity_event.add_callback(
          std::bind(&HybridPositioner::new_inertial_position_callback, this, std::placeholders::_1))},
      uwb_pos_event_id{arwain::Events::new_arwain_velocity_event.add_callback(
          std::bind(&HybridPositioner::new_uwb_position_callback, this, std::placeholders::_1))},
      imu_rot_event_id{arwain::Events::new_orientation_data_event.add_callback(
          std::bind(&HybridPositioner::new_orientation_data_callback, this, std::placeholders::_1))}
{
    ServiceManager::register_service(this, HybridPositioner::service_name);
    init();
}

HybridPositioner::~HybridPositioner()
{
    // TODO Should be easy enough to create an RAII type for registrations that won't need explicit deletion.
    arwain::Events::new_arwain_velocity_event.remove_callback(imu_vel_event_id);
    arwain::Events::new_uwb_position_event.remove_callback(uwb_pos_event_id);
    arwain::Events::new_orientation_data_event.remove_callback(imu_rot_event_id);
    arwain::Events::new_arwain_position_event.remove_callback(imu_pos_event_id);
    ServiceManager::unregister_service(HybridPositioner::service_name);
}

void HybridPositioner::new_inertial_velocity_callback(arwain::Events::Vector3EventWithDt inertial_velocity)
{
    hyb.position = hyb.position + inertial_velocity.velocity * inertial_velocity.dt;
}

void HybridPositioner::new_inertial_position_callback(arwain::Events::Vector3EventWithDt inertial_event)
{
    inertial_positions.push_back(inertial_event.position);
}

void HybridPositioner::new_uwb_position_callback(arwain::Events::Vector3EventWithDt uwb_event)
{
    // TODO Need to think about the dt since frequency of updates is unknown and irregular.
    uwb_positions.push_back(uwb_event.position);
    hyb.position = hyb.position - arwain::config.hybrid_position_gain * (hyb.position - uwb_event.position);
}

void HybridPositioner::new_orientation_data_callback(arwain::Events::RotorEventWithDt rotor_data)
{
    
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
        hybrid_pos_log << loop_scheduler.count() << ' ' << hyb.position.x << ' ' << hyb.position.y << ' ' << hyb.position.z << '\n';
        loop_scheduler.await();
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
