#include <iomanip>
#include <thread>
#include <cstring>

#include "velocity_prediction.hpp"
#include "logger.hpp"
#include "arwain.hpp"

#include <zmq.h>

static std::string inference_tcp_socket = "tcp://*:5555";

/** \brief This has only been developed to the proof of concept stage and is not suitable for deployment.
 */
void predict_velocity()
{
    if (arwain::config.no_inference)
    {
        return;
    }
    // *****************************************************************//
    // NOTE This has only been developed to the proof of concept stage. //
    // It will not work fully without additional work.                  //
    // *****************************************************************//

    // Wait for enough time to ensure the IMU buffer contains valid and useful data before starting.
    std::chrono::milliseconds presleep(1000);
    std::this_thread::sleep_for(presleep*10);

    // Set up socket
    void *context = zmq_ctx_new();
    void *responder = zmq_socket(context, ZMQ_REP);
    zmq_bind(responder, inference_tcp_socket.c_str());

    // Buffer to contain local copy of IMU data.
    std::deque<vector6> imu;

    vector3 position{0, 0, 0};
    vector3 velocity{0, 0, 0};

    // Request and response buffers.
    std::stringstream request;
    char response_buffer[50];

    // Open files for logging.
    // File handles for logging.
    arwain::Logger position_file;
    arwain::Logger velocity_file;
    if (arwain::config.log_to_file)
    {
        velocity_file.open(arwain::folder_date_string + "/velocity.txt");
        position_file.open(arwain::folder_date_string + "/position.txt");
        velocity_file << "time x y z" << "\n";
        position_file << "time x y z" << "\n";
    }

    // Set up timing.
    std::chrono::time_point<std::chrono::system_clock> lastTime = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{arwain::Intervals::VELOCITY_PREDICTION_INTERVAL};

    while (!arwain::shutdown)
    {
        { // Grab latest IMU packet
            std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
            imu = arwain::Buffers::IMU_WORLD_BUFFER;
        }

        // Check what the time really is since it might not be accurate.
        time = std::chrono::system_clock::now();

        // Get dt in seconds since last udpate, and update lastTime.
        double dt = std::chrono::duration_cast<std::chrono::milliseconds>(time - lastTime).count()/1000.0;
        lastTime = time;
        
        // Load the IMU data into a string for serial transmission, gyro first.
        for (unsigned int i = 0; i < imu.size(); i++)
        {
            request << std::setprecision(15) << (float)imu[i].gyro.x << ",";
            request << std::setprecision(15) << (float)imu[i].gyro.y << ",";
            request << std::setprecision(15) << (float)imu[i].gyro.z << ",";
            request << std::setprecision(15) << (float)imu[i].acce.x << ",";
            request << std::setprecision(15) << (float)imu[i].acce.y << ",";
            request << std::setprecision(15) << (float)imu[i].acce.z << ",";
        }

        // Send the data and await response.
        std::string fromStream = request.str();
        const char *str = fromStream.c_str();
        zmq_send(responder, str, strlen(str), 0);
        zmq_recv(responder, response_buffer, 50, 0);
        request.str("");

        // Process the answer buffer into local velocity buffers.
        // Assume a comma-separated list of three floats.
        std::string answer{response_buffer};
        if (answer == "accept")
        {
            continue;
        }
        int delimiter = answer.find(",");
        std::stringstream(answer.substr(0, delimiter)) >> velocity.x;
        answer = answer.substr(delimiter+1);
        delimiter = answer.find(",");
        std::stringstream(answer.substr(0, delimiter)) >> velocity.y;
        std::stringstream(answer.substr(delimiter+1)) >> velocity.z;

        { // Store velocity in global buffer.
            std::lock_guard<std::mutex> lock{arwain::Locks::VELOCITY_BUFFER_LOCK};
            arwain::Buffers::VELOCITY_BUFFER.pop_front();
            arwain::Buffers::VELOCITY_BUFFER.push_back(velocity);
        }

        // Compute new position.
        velocity = {
            std::cos(arwain::yaw_offset)*velocity.x - std::sin(arwain::yaw_offset)*velocity.y,
            std::sin(arwain::yaw_offset)*velocity.x + std::cos(arwain::yaw_offset)*velocity.y,
            velocity.z
        };
        position = position + dt * velocity;

        { // Add new position to global buffer.
            std::lock_guard<std::mutex> lock{arwain::Locks::POSITION_BUFFER_LOCK};
            arwain::Buffers::POSITION_BUFFER.pop_front();
            arwain::Buffers::POSITION_BUFFER.push_back(position);
        }

        // Log results to file.
        if (arwain::config.log_to_file)
        {
            velocity_file << time.time_since_epoch().count() << " " << velocity.x << " " << velocity.y << " " << velocity.z << "\n";
            position_file << time.time_since_epoch().count() << " " << position.x << " " << position.y << " " << position.z << "\n";
        }

        // Wait until next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }
    
    // Instruct the NCS2 interface script to quit.
    zmq_send(responder, "stop", strlen("stop"), 0);

    // Flush and close log files.
    if (arwain::config.log_to_file)
    {
        velocity_file.close();
        position_file.close();
    }
}
