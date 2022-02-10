#include <iomanip>
#include <thread>
#include <cstring>
#include <eigen3/Eigen/Dense>

#include "velocity_prediction.hpp"
#include "logger.hpp"
#include "arwain.hpp"

#include <zmq.h>

static std::string inference_tcp_socket = "tcp://*:5555";

#define DEBUGKALMAN

class VelocityKalmanFilter
{
    private:
        uint64_t last_time = 0;
        Eigen::Matrix<double, 3, 3> kalman_gain;
        Eigen::Matrix<double, 3, 1> state{0, 0, 0}; // Z
        Eigen::Matrix<double, 3, 3> state_transition; // A
        Eigen::Matrix<double, 3, 3> control_matrix; // B
        Eigen::Matrix<double, 3, 1> process_noise; // omega
        Eigen::Matrix<double, 3, 1> measurement_noise; // omega
        Eigen::Matrix<double, 3, 1> gravity{0, 0, 9.81}; // gravity
        Eigen::Matrix<double, 3, 3> process_covariance;
        Eigen::Matrix<double, 3, 3> measurement_covariance;
        Eigen::Matrix<double, 3, 3> initial_process_covariance;

        // Fitting matrices
        Eigen::Matrix<double, 3, 3> I{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        Eigen::Matrix<double, 3, 3> C = I;
        Eigen::Matrix<double, 3, 3> H = I;

        arwain::Logger log;

    public:
        VelocityKalmanFilter(double process_uncertainty, double measurement_uncertainty)
        {
            state = Eigen::Matrix<double, 3, 1>{0, 0, 0};
            process_noise = Eigen::Matrix<double, 3, 1>{0, 0, 0};
            auto& p = process_uncertainty;
            process_covariance = Eigen::Matrix<double, 3, 3>{
                {p*p, 0,     0},
                {0,   p*p,   0},
                {0,   0,   p*p}
            };
            initial_process_covariance = process_covariance;
            measurement_noise = Eigen::Matrix<double, 3, 1>{0, 0, 0};
            auto& m = measurement_uncertainty;
            measurement_covariance = Eigen::Matrix<double, 3, 3>{
                {m*m,0,0},
                {0,m*m,0},
                {0,0,m*m}
            };
            log.open("kalman_log.txt");
        }

        Vector3 update(const Vector3& velocity_measurement, const Vector3& average_acceleration)
        {
            #ifdef DEBUGKALMAN
            static int updatecount = 0;
            #endif
            if (last_time == 0)
            {
                last_time = std::chrono::system_clock::now().time_since_epoch().count();
                return velocity_measurement;
            }
            auto now = std::chrono::system_clock::now().time_since_epoch().count();
            double dt = (now - last_time) / 1e9;
            last_time = now;

            #ifdef DEBUGKALMAN
            log << "Acceleration supplied:\n";
            log << average_acceleration << "\n";
            log << "Modified acceleration:\n";
            log << average_acceleration - Vector3{0, 0, 9.81} << "\n";
            log << "Velocity measurement:\n";
            log << velocity_measurement << "\n";
            #endif

            state_transition = I;
            control_matrix = Eigen::Matrix<double, 3, 3>{{dt, 0, 0},{0, dt, 0},{0, 0, dt}};

            #ifdef DEBUGKALMAN
            log << "State pre-update:\n";
            log << state << "\n";
            #endif

            Eigen::Matrix<double, 3, 1> acceleration{average_acceleration.x, average_acceleration.y, average_acceleration.z};
            
            /*
                X = A*X + B*u + w
            */
            // The prediction step updates the state according to the model and measurements from the accelerometer.
            state = state_transition * state + control_matrix * (acceleration - gravity) + process_noise;

            #ifdef DEBUGKALMAN
            log << "State after state transition:" << "\n";
            log << state << "\n";
            #endif

            #ifdef DEBUGKALMAN
            log << "Process covariance pre-update 1:" << "\n";
            log << process_covariance << "\n";
            #endif

            /*
                P = A*P*A.T + Q
            */
            // The process covariance matrix is updated to prevent it approaching zero.
            process_covariance = state_transition * process_covariance * state_transition.transpose() + initial_process_covariance;

            #ifdef DEBUGKALMAN
            log << "Process covariance post-update 1:" << "\n";
            log << process_covariance << "\n";
            #endif

            #ifdef DEBUGKALMAN
            log << "Kalman gain pre-update:" << "\n";
            log << kalman_gain << "\n";
            #endif

            /*
                K = P*H * (H*P*H.T + R)^{-1}
            */
            // We update the Kalman gain based on the new process covariance.
            kalman_gain = (process_covariance * H) * (H * process_covariance * H.transpose() + measurement_covariance).inverse();

            #ifdef DEBUGKALMAN
            log << "Kalman gain post-update:" << "\n";
            log << kalman_gain << "\n";
            #endif

            // Modify the state according to the measurement and the Kalman gain.
            /*
                Y = C*Y + z
            */
            Eigen::Matrix<double, 3, 1> measurement = C * Eigen::Matrix<double, 3, 1>{velocity_measurement.x, velocity_measurement.y, velocity_measurement.z} + measurement_noise;

            /*
                X = X + K*(Y - X)
            */
            state = state + kalman_gain * (measurement - H * state);

            #ifdef DEBUGKALMAN
            log << "State post measurement update:" << "\n";
            log << state << "\n";
            #endif

            // Modify the process covariance according to the Kalman gain.
            /*
                P = (I - H*K) * P
            */
            process_covariance = (I - H * kalman_gain) * process_covariance;

            #ifdef DEBUGKALMAN
            log << "Process covariance post-update 2:" << "\n";
            log << process_covariance << "\n";
            #endif

            #ifdef DEBUGKALMAN
            updatecount++;
            #endif

            return {state[0], state[1], state[2]};
        }
};


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
    std::deque<Vector6> imu;


    // Request and response buffers.
    std::stringstream request;
    char response_buffer[50];

    arwain::ready_for_inference = true;

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::Inference:
            {
                Vector3 position{0, 0, 0};
                Vector3 kalman_position{0, 0, 0};
                Vector3 velocity{0, 0, 0};
                
                VelocityKalmanFilter kalman{0.0001, 0.9}; // process, measurement

                // Open files for logging.
                // File handles for logging.
                arwain::Logger velocity_file;
                arwain::Logger kalman_velocity_file;
                arwain::Logger position_file;
                arwain::Logger kalman_position_file;

                if (arwain::config.log_to_file)
                {
                    velocity_file.open(arwain::folder_date_string + "/velocity.txt");
                    kalman_velocity_file.open(arwain::folder_date_string + "/kalman_velocity.txt");
                    position_file.open(arwain::folder_date_string + "/position.txt");
                    kalman_position_file.open(arwain::folder_date_string + "/kalman_position.txt");
                    velocity_file << "time x y z" << "\n";
                    kalman_velocity_file << "time x y z" << "\n";
                    position_file << "time x y z" << "\n";
                    kalman_position_file << "time x y z" << "\n";
                }

                // Set up timing.
                std::chrono::time_point<std::chrono::system_clock> lastTime = std::chrono::system_clock::now();
                std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now();
                std::chrono::milliseconds interval{arwain::Intervals::VELOCITY_PREDICTION_INTERVAL};

                while (arwain::system_mode == arwain::OperatingMode::Inference)
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

                    Vector3 average_acceleration{0, 0, 0};
                    for (std::deque<Vector6>::iterator it = imu.end() - 10; it != imu.end(); ++it)
                    {
                        average_acceleration = average_acceleration + (*it).acce;
                    }
                    average_acceleration = average_acceleration / 10.0;

                    auto kalman_velocity = kalman.update(velocity, average_acceleration);

                    { // Store velocity in global buffer.
                        std::lock_guard<std::mutex> lock{arwain::Locks::VELOCITY_BUFFER_LOCK};
                        arwain::Buffers::VELOCITY_BUFFER.pop_front();
                        arwain::Buffers::VELOCITY_BUFFER.push_back(velocity);
                    }

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
                    kalman_position = kalman_position + dt * kalman_velocity;

                    { // Add new position to global buffer.
                        std::lock_guard<std::mutex> lock{arwain::Locks::POSITION_BUFFER_LOCK};
                        arwain::Buffers::POSITION_BUFFER.pop_front();
                        arwain::Buffers::POSITION_BUFFER.push_back(position);
                    }

                    // Log results to file.
                    if (arwain::config.log_to_file)
                    {
                        velocity_file << time.time_since_epoch().count() << " " << velocity.x << " " << velocity.y << " " << velocity.z << "\n";
                        kalman_velocity_file << time.time_since_epoch().count() << " " << kalman_velocity.x << " " << kalman_velocity.y << " " << kalman_velocity.z << "\n";
                        position_file << time.time_since_epoch().count() << " " << position.x << " " << position.y << " " << position.z << "\n";
                        kalman_position_file << time.time_since_epoch().count() << " " << kalman_position.x << " " << kalman_position.y << " " << kalman_position.z << "\n";
                    }

                    // Wait until next tick.
                    time = time + interval;
                    std::this_thread::sleep_until(time);
                }
                
                // Flush and close log files.
                if (arwain::config.log_to_file)
                {
                    velocity_file.close();
                    kalman_velocity_file.close();
                    position_file.close();
                    kalman_position_file.close();
                }
                break;
            }
            default:
            {
                sleep_ms(10);
                break;
            }
        }
    }
    
    // Instruct the NCS2 interface script to quit.
    zmq_send(responder, "stop", strlen("stop"), 0);
}
