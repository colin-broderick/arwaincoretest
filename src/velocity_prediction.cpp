#include "velocity_prediction.h"
#include "logger.h"

static std::string inference_tcp_socket = "tcp://*:5555";

#if USE_SOCKET_INFERENCE
/** \brief This has only been developed to the proof of concept stage and is not suitable for deployment.
 */
void predict_velocity()
{
    if (NO_INFERENCE)
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
    std::deque<std::array<double, 6>> imu;

    std::array<double, 3> position{0, 0, 0};
    std::array<double, 3> velocity{0, 0, 0};

    // Request and response buffers.
    std::stringstream request;
    char response_buffer[50];

    // Open files for logging.
    // File handles for logging.
    arwain::Logger position_file;
    arwain::Logger velocity_file;
    if (LOG_TO_FILE)
    {
        velocity_file.open(FOLDER_DATE_STRING + "/velocity.txt");
        position_file.open(FOLDER_DATE_STRING + "/position.txt");
        velocity_file << "# time x y z" << "\n";
        position_file << "# time x y z" << "\n";
    }

    // Set up timing.
    std::chrono::time_point<std::chrono::system_clock> lastTime = std::chrono::system_clock::now();
    std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{VELOCITY_PREDICTION_INTERVAL};

    while (!SHUTDOWN)
    {
        { // Grab latest IMU packet
            std::lock_guard<std::mutex> lock{IMU_BUFFER_LOCK};
            imu = IMU_WORLD_BUFFER;
        }

        // Check what the time really is since it might not be accurate.
        time = std::chrono::system_clock::now();

        // Get dt in seconds since last udpate, and update lastTime.
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(time - lastTime).count()/1000.0;
        lastTime = time;
        
        // Load the IMU data into a string for serial transmission.
        for (unsigned int i = 0; i < imu.size(); i++)
        {
            for (unsigned int j = 0; j < 6; j++)
            {
                request << std::setprecision(15) << (float)imu[i][(j+3)%6] << ",";
            }
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
        std::stringstream(answer.substr(0, delimiter)) >> velocity[0];
        answer = answer.substr(delimiter+1);
        delimiter = answer.find(",");
        std::stringstream(answer.substr(0, delimiter)) >> velocity[1];
        std::stringstream(answer.substr(delimiter+1)) >> velocity[2];

        { // Store velocity in global buffer.
            std::lock_guard<std::mutex> lock{VELOCITY_BUFFER_LOCK};
            VELOCITY_BUFFER.pop_front();
            VELOCITY_BUFFER.push_back(velocity);
        }

        // Compute new position.
        position[0] = position[0] + dt * velocity[0];
        position[1] = position[1] + dt * velocity[1];
        position[2] = position[2] + dt * velocity[2];

        { // Add new position to global buffer.
            std::lock_guard<std::mutex> lock{POSITION_BUFFER_LOCK};
            POSITION_BUFFER.pop_front();
            POSITION_BUFFER.push_back(position);
        }

        // Log results to file.
        if (LOG_TO_FILE)
        {
            velocity_file << time.time_since_epoch().count() << " " << velocity[0] << " " << velocity[1] << " " << velocity[2] << "\n";
            position_file << time.time_since_epoch().count() << " " << position[0] << " " << position[1] << " " << position[2] << "\n";
        }

        // Wait until next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }
    
    // Instruct the NCS2 interface script to quit.
    zmq_send(responder, "stop", strlen("stop"), 0);

    // Flush and close log files.
    if (LOG_TO_FILE)
    {
        velocity_file.close();
        position_file.close();
    }
}

#else

/** \brief Periodically makes velocity predictions based on data buffers, and adds that velocity and thereby position to the relevant buffers.
 */
void predict_velocity()
{
    // // Skip inference if command line says so.
    // if (!NO_INFERENCE)
    // {
    //     // TODO: Merge the inference code into this function. Will need further abstraction?
    //     // TODO: Set up NPU and feed in model.
    //     // TODO: Make it possible to specify the model file path, and fail gracefully if not found.
    //     // Torch model{"./xyzronin_v0-5_all2D_small.pt", {1, 6, 200, 1}};
    //     arwain::Torch model{"./xyzronin_v0-6.pt", {1, 6, 200}};

    //     // Wait for enough time to ensure the IMU buffer contains valid and useful data before starting.
    //     std::chrono::milliseconds presleep(1000);
    //     std::this_thread::sleep_for(presleep*3);

    //     // Set up timing.
    //     auto time = std::chrono::system_clock::now();
    //     std::chrono::milliseconds interval{VELOCITY_PREDICTION_INTERVAL};

    //     // Initialize buffers to contain working values.
    //     std::array<double, 3> vel;                        // The sum of npu_vel and imu_vel_delta.
    //     std::array<double, 3> npu_vel;                    // To hold the neural network prediction of velocity.
    //     std::array<double, 3> vel_previous = {0, 0, 0};   // Contains the velocity calculation from the previous loop.
    //     std::array<double, 3> npu_vel_delta;              // Stores the difference between the npu velocity prediction and vel_previous.
    //     std::array<double, 3> imu_vel_delta;              // To integrate the velocity based on IMU readings.
    //     std::array<double, 3> position;
    //     std::deque<std::array<double, 6>> imu;            // To contain the last second of IMU data.
    //     std::deque<std::array<double, 6>> imu_latest;     // To contain the last VELOCITY_PREDICTION_INTERVAL of IMU data.

    //     // File handles for logging.
    //     arwain::Logger position_file;
    //     arwain::Logger velocity_file;

    //     // Time in seconds between inferences.
    //     double interval_seconds = ((double)(VELOCITY_PREDICTION_INTERVAL))/1000.0;

    //     // TEST How far back to look in the IMU buffer for integration.
    //     int backtrack = (int)((1000/IMU_READING_INTERVAL)*interval_seconds);

    //     // Open files for logging.
    //     if (LOG_TO_FILE)
    //     {
    //         velocity_file.open(FOLDER_DATE_STRING + "/velocity.txt");
    //         position_file.open(FOLDER_DATE_STRING + "/position.txt");
    //         velocity_file << "# time x y z" << "\n";
    //         position_file << "# time x y z" << "\n";
    //     }
        
    //     while (!SHUTDOWN)
    //     {
    //         { // Grab latest IMU packet
    //             std::lock_guard<std::mutex> lock{IMU_BUFFER_LOCK};
    //             imu = IMU_WORLD_BUFFER;
    //         }

    //         // TEST Make velocity prediction
    //         std::vector<double> v = model.infer(imu);
    //         npu_vel = {v[0], v[1], v[2]};
    //         // std::cout << v[0] << "," << v[1] << "," << v[2] << std::endl;
    //         // TEST Find the change in velocity from the last period, as predicted by the npu.
    //         npu_vel_delta = npu_vel - vel_previous;

    //         // TEST Get last interval worth of IMU data.
    //         imu_latest = {imu.end() - backtrack, imu.end()};

    //         // TEST Single integrate the small IMU slice to get delta-v over the period.
    //         imu_vel_delta = integrate(imu_latest, interval_seconds );

    //         // TEST Weighted combination of velocity deltas from NPU and IMU integration.
    //         // TODO This gives very wrong results when trusting the IMU at all. Investigate and repair (or bin it).
    //         vel = npu_vel_delta*CONFIG.npu_vel_weight_confidence + imu_vel_delta*(1-CONFIG.npu_vel_weight_confidence);

    //         // TEST Add the filtered delta onto the previous vel estimate and add to buffer.
    //         vel = vel + vel_previous;

    //         {
    //             std::lock_guard<std::mutex> lock{VELOCITY_BUFFER_LOCK};
    //             VELOCITY_BUFFER.pop_front();
    //             VELOCITY_BUFFER.push_back(vel);
    //         }

    //         // TEST Store the velocity for use in the next loop (saves having to access the buffer for a single element).
    //         vel_previous = vel;

    //         // Iterate velocity onto position to get new position.
    //         position[0] = position[0] + interval_seconds * vel[0];
    //         position[1] = position[1] + interval_seconds * vel[1];
    //         position[2] = position[2] + interval_seconds * vel[2];
            
    //         { // Update position buffer.
    //             std::lock_guard<std::mutex> lock{POSITION_BUFFER_LOCK};
    //             POSITION_BUFFER.pop_front();
    //             POSITION_BUFFER.push_back(position);
    //         }

    //         // Add position and velocity data to file.
    //         if (LOG_TO_FILE)
    //         {
    //             velocity_file << time.time_since_epoch().count() << " " << vel[0] << " " << vel[1] << " " << vel[2] << "\n";
    //             position_file << time.time_since_epoch().count() << " " << position[0] << " " << position[1] << " " << position[2] << "\n";
    //         }

    //         // Wait until next tick.
    //         time = time + interval;
    //         std::this_thread::sleep_until(time);
    //     }

    //     // Close file handle(s).
    //     if (LOG_TO_FILE)
    //     {
    //         velocity_file.close();
    //         position_file.close();
    //     }
    // }
}

/** \brief Multiple double 3-array by scalar value. */
template <typename U> static std::array<double, 3> operator*(const std::array<double, 3>& a1, U scalar)
{
  std::array<double, 3> a;
  for (typename std::array<double, 3>::size_type i = 0; i < a1.size(); i++)
    a[i] = a1[i]*scalar;
  return a;
}

/** \brief Element-wise difference of two double 3-arrays. */
static std::array<double, 3> operator-(const std::array<double, 3>& a1, const std::array<double, 3>& a2)
{
  std::array<double, 3> a;
  for (typename std::array<double, 3>::size_type i = 0; i < a1.size(); i++)
    a[i] = a1[i] - a2[i];
  return a;
}

/** \brief Element-wise sum of two double 3-arrays. */
static std::array<double, 3> operator+(const std::array<double, 3>& a1, const std::array<double, 3>& a2)
{
  std::array<double, 3> a;
  for (typename std::array<double, 3>::size_type i = 0; i < a1.size(); i++)
    a[i] = a1[i] + a2[i];
  return a;
}

/** \brief Integrates acceleration data over a small window to generate velocity delta.
 * \param data The buffer to integrate over.
 * \param dt The time delta between buffer values.
 * \param offset The position in the buffer entries of the first acceleration value.
 * \return An array of x, y, z velocities.
 */
static std::array<double, 3> integrate(std::deque<std::array<double, 6>> &data, double dt, unsigned int offset = 0)
{
    // TODO What the hell was I thinking here? This definitely won't work.
    std::array<double, 3> acc_mean = {-0.182194123, -0.59032666517, 9.86202363151991};
    std::array<double, 3> integrated_data = {0, 0, 0};
    for (unsigned int i = 0; i < data.size(); i++)
    {
        for (unsigned int j = 0; j < 3; j++)
        {
            integrated_data[j] = integrated_data[j] + (data[i][j + offset] - acc_mean[j]) * dt;
        }
    }
    return integrated_data;
}

#endif
