#include <iostream>

#include <arwain/icp.hpp>

#include "arwain/hybrid_positioner.hpp"
#include "arwain/velocity_prediction.hpp"
#include "arwain/uwb_reader.hpp"
#include "arwain/service_manager.hpp"
#include "arwain/events.hpp"
#include "arwain/global_buffer.hpp"

/*
Keep 10-second record of pure uwb position
Keep 10-second record of pure inertial position.
Remap both so originate at (0, 0, 0).
Do ICP to deduce best rotation of inertial onto UWB.
All new points rotated by current_angular_correction.
Current_angular_correction is updated every frame.
*/

/** \brief These buffers keep a short record of a subsample of positions from these two sources. */
namespace
{
    GlobalBuffer<Vector3, 20> inertial_positions;
    GlobalBuffer<Vector3, 20> uwb_positions;
    double time_between_pushes = 0.5 /* seconds */;
}

HybridPositioner::HybridPositioner()
    : imu_vel_event_id{arwain::Events::new_arwain_velocity_event.add_callback(
          std::bind(&HybridPositioner::new_inertial_velocity_callback, this, std::placeholders::_1))},
      imu_pos_event_id{arwain::Events::new_arwain_position_event.add_callback(
          std::bind(&HybridPositioner::new_inertial_position_callback, this, std::placeholders::_1))},
      uwb_pos_event_id{arwain::Events::new_uwb_position_event.add_callback(
          std::bind(&HybridPositioner::new_uwb_position_callback, this, std::placeholders::_1))},
      imu_rot_event_id{arwain::Events::new_orientation_data_event.add_callback(
          std::bind(&HybridPositioner::new_orientation_data_callback, this, std::placeholders::_1))}
{
    ServiceManager::register_service(this, HybridPositioner::service_name);
    core_setup();
    job_thread = std::jthread{std::bind_front(&HybridPositioner::run, this)};
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

void HybridPositioner::new_inertial_velocity_callback(arwain::Events::Vector3EventWithDt inertial_event)
{
    if (arwain::config.hybrid_heading_compute)
    {   // Rotate the velocity vector according to current best guess heading correction, then integrate velocity.
        auto rotor = Quaternion{current_angular_correction, Vector3::Axis::Z.to_array()};
        auto rotated_velocity = arwain::apply_quat_rotor_to_vector3(inertial_event.velocity, rotor);
        hyb.position = hyb.position + rotated_velocity * inertial_event.dt;
    }
    else
    {   // Integrate raw velocity - this is equivalent to standard inertial navigation.
        hyb.position = hyb.position + inertial_event.velocity * inertial_event.dt;
    }
}

void HybridPositioner::new_inertial_position_callback(arwain::Events::Vector3EventWithDt inertial_event)
{
    // This might seem like a mistake since I am checking heading config in the inertial position callback.
    // It is correct. Heading correction is what the inertial positions are used for in the hybrid localizer.
    if (!arwain::config.hybrid_heading_compute)
    {
        return;
    }

    static double time_since_push = 0;
    time_since_push += inertial_event.dt;
    if (time_since_push > time_between_pushes)
    {
        inertial_positions.push_back(inertial_event.position);
        time_since_push -= time_between_pushes;
    }
}

static bool uwb_positions_supplied = false;

void HybridPositioner::new_uwb_position_callback(arwain::Events::Vector3EventWithDt uwb_event)
{
    uwb_positions_supplied = true;

    if (!arwain::config.hybrid_position_compute)
    {
        return;
    }

    static double time_since_push = 0;
    time_since_push += uwb_event.dt;
    if (time_since_push > time_between_pushes)
    {
        uwb_positions.push_back(uwb_event.position);
        time_since_push -= time_between_pushes;
    }

    hyb.position = hyb.position - arwain::config.hybrid_position_gain * (hyb.position - uwb_event.position) * uwb_event.dt;
}

void HybridPositioner::new_orientation_data_callback(arwain::Events::RotorEventWithDt rotor_data)
{
    
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

std::vector<PositionICP> gbuffer_to_icp(const GlobalBuffer<Vector3, 20>& gbuffer)
{
    std::vector<PositionICP> icp_buffer;
    for (auto& element : gbuffer.get_data())
    {
        icp_buffer.push_back({element.x, element.y, element.z});
    }
    return icp_buffer;
}

std::optional<ICPResult> arwain_icp_wrapper(const GlobalBuffer<Vector3, 20>& inertial_positions, const GlobalBuffer<Vector3, 20>& uwb_positions)
{
    // TODO This wrapper business can all be eliminated if we have arwain_icp take sensible types.
    auto icp_inertial_positions = gbuffer_to_icp(inertial_positions);
    auto icp_uwb_positions = gbuffer_to_icp(uwb_positions);
    try
    {
        return arwain_icp_2d(icp_inertial_positions, icp_uwb_positions);
    }
    catch (const std::exception& e)
    {
        std::cout << "Error running ICP: " << e.what() << '\n';
        return std::nullopt;
    }
}

constexpr double degrees_0 = 0;
constexpr double degrees_180 = 3.14159265359;
constexpr double degrees_90 = degrees_180 / 2.0;
constexpr double degrees_270 = degrees_180 * 3.0 / 2.0;
constexpr double degrees_360 = degrees_0;

/** \brief If the computed correction is more than 90° different form the old one, we have probably
 * computed the smaller wrong angle instead of the large right angle. Subtract 180° from the proposed
 * correction to point back in the right direction.
 */
auto update_correction(double new_angle, double old_angle)
{
    if (std::abs(new_angle - old_angle) > degrees_90)
    {
        return new_angle - degrees_180;
    }
    else
    {
        return new_angle;
    }
}

bool uwb_safe()
{
    static bool is_safe = false;
    if (is_safe)
    {
        return is_safe;
    }

    for (auto& pos : uwb_positions.get_data())
    {
        if (pos != Vector3::Zero)
        {
            is_safe = true;
            return is_safe;
        }
    }
    return is_safe;
}

void HybridPositioner::run_inference()
{
    // The things I do here probably seem pointless but matching the pattern
    // of the other ArwainJobs was the easiest way to get the job done. Changes will
    // need to consider a refactor of the whole ArwainJob inheritance pattern.

    setup_inference();

    Timers::IntervalTimer loop_scheduler{100_ms, "hybrid_position_timer"};

    auto loop_count = 1;
    while (mode == arwain::OperatingMode::Inference)
    {
        if (!arwain::config.hybrid_heading_compute)
        {
            loop_scheduler.await();
            continue;
        }

        if (!uwb_safe())
        {
            loop_scheduler.await();
            continue;
        }

        if (loop_count % 50 == 0)
        {
            auto new_angular_correction = arwain_icp_wrapper(inertial_positions, uwb_positions);
            if (new_angular_correction && new_angular_correction.value().confidence < 1.0)
            {
                std::cout << new_angular_correction.value().angle << " " << new_angular_correction.value().confidence << '\n';
                current_angular_correction = update_correction(new_angular_correction.value().angle, current_angular_correction);
            }
        }
        hybrid_pos_log << loop_scheduler.count() << ' ' << hyb.position.x << ' ' << hyb.position.y << ' ' << hyb.position.z << '\n';
        loop_count++;
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
