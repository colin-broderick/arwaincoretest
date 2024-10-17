#include <iomanip>
#include <thread>
#include <cstring>
#include <eigen3/Eigen/Dense>

#include <arwain/event_manager.hpp> 

#include "arwain/velocity_prediction.hpp"
#include "arwain/logger.hpp"
#include "arwain/arwain.hpp"
#include "arwain/exceptions.hpp"
#include "arwain/service_manager.hpp"
#include "arwain/velocity_prediction.hpp"
#include "arwain/service_manager.hpp"
#include "arwain/uwb_reader.hpp"
#include "arwain/hybrid_positioner.hpp"
#include "arwain/tf_inferrer.hpp"

PositionVelocityInference::PositionVelocityInference()
{
    ServiceManager::register_service(this, service_name);
    core_setup();
    if (job_thread.joinable())
    {
        job_thread.join();
    }
    inferrer = std::make_unique<TFInferrer>(arwain::config.inference_model_path.c_str());
    job_thread = std::jthread{std::bind_front(&PositionVelocityInference::run, this)};
    pin_thread(job_thread, 1);
}

PositionVelocityInference::~PositionVelocityInference()
{
    ServiceManager::unregister_service(service_name);
}

void PositionVelocityInference::core_setup()
{
    // Wait for enough time to ensure the IMU buffer contains valid and useful data before starting.
    std::this_thread::sleep_for(1000_ms);
}

void PositionVelocityInference::run()
{
    if (arwain::config.no_inference)
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

void PositionVelocityInference::run_idle()
{
    sleep_ms(10);
}

void PositionVelocityInference::setup_inference()
{
    position = {0, 0, 1.5};
    velocity = {0, 0, 0};
    velocity_file.open(arwain::folder_date_string + "/velocity.txt");
    position_file.open(arwain::folder_date_string + "/position.txt");
    velocity_file << "time x y z" << "\n";
    position_file << "time x y z" << "\n";
}

// TODO Move all the tensorflow stuff out of the header and into this file.
// Should dramatically improve build times. Currently anything that includes
// the header has to deal with all the tensorflow shit.

static Vector3 calculate_average_acceleration(std::deque<ImuData>& imu_data)
{
    Vector3 average_acceleration{0, 0, 0};
    for (std::deque<ImuData>::iterator it = imu_data.end() - 10; it != imu_data.end(); ++it)
    {
        average_acceleration = average_acceleration + (*it).acce;
    }
    return average_acceleration / 10.0;
}

void PositionVelocityInference::run_inference()
{
    setup_inference();

    // Set up timing.
    // TODO Use JobInterval if possible; the timing here is different to other threads.
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> time = std::chrono::high_resolution_clock::now();
    constexpr auto interval = arwain::Intervals::VELOCITY_PREDICTION_INTERVAL;

    while (mode == arwain::OperatingMode::Inference)
    {
        auto imu_data = arwain::Buffers::IMU_WORLD_BUFFER.get_data();
        // Check what the time really is since it might not be accurate.
        time = std::chrono::high_resolution_clock::now();
        // Get dt in seconds since last udpate, and update lastTime.
        const double dt = std::chrono::duration_cast<std::chrono::milliseconds>(time - last_time).count()/1000.0;
        last_time = time;

        velocity = inferrer->infer(imu_data);

        // TODO Temporary to suppress errors in vertical velocity inference;
        // Remove when root issue resolved.
        if (std::abs(velocity.z) > 4.0)
        {
            velocity.z = 0.0;
        }
        // Feed the activity metric.
        // TODO Maybe this can be pulled out and an event triggered instead.
        arwain::activity_metric.feed_velocity(velocity);

        auto average_acceleration = calculate_average_acceleration(imu_data);

        arwain::Buffers::VELOCITY_BUFFER.push_back(velocity);

        // Compute new position.
        if (arwain::config.correct_with_yaw_diff)
        {
            velocity = {
                std::cos(-arwain::yaw_offset)*velocity.x - std::sin(-arwain::yaw_offset)*velocity.y,
                std::sin(-arwain::yaw_offset)*velocity.x + std::cos(-arwain::yaw_offset)*velocity.y,
                velocity.z
            };
        }

        position = position + dt * velocity;
        arwain::Events::new_arwain_position_event.invoke({position, dt});
        arwain::Events::new_arwain_velocity_event.invoke({velocity, dt});

        arwain::Buffers::POSITION_BUFFER.push_back(position);

        // Log results to file.
        velocity_file << time.time_since_epoch().count() << " " << velocity.x << " " << velocity.y << " " << velocity.z << "\n";
        position_file << time.time_since_epoch().count() << " " << position.x << " " << position.y << " " << position.z << "\n";

        last_inference_time = time.time_since_epoch().count();

        // Wait until next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);

    }

    cleanup_inference();
}

bool PositionVelocityInference::cleanup_inference()
{
    position = {0, 0, 1.5};
    velocity = {0, 0, 0};
    return position_file.close() && velocity_file.close();
}

bool PositionVelocityInference::join()
{
    if (job_thread.joinable())
    {
        job_thread.join();
        return true;
    }
    return false;
}

Vector3 PositionVelocityInference::get_position() const
{
    return position;
}
