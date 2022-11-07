#include <iomanip>
#include <thread>
#include <cstring>
#include <eigen3/Eigen/Dense>

#include "velocity_prediction.hpp"
#include "logger.hpp"
#include "arwain.hpp"

#if USENCS2
    #include <zmq.h>
#else // Using tflite inference
    #include "tensorflow/lite/interpreter.h"
    #include "tensorflow/lite/kernels/register.h"
    #include "tensorflow/lite/model.h"
    #include "tensorflow/lite/tools/gen_op_registration.h"
#endif

static std::string inference_tcp_socket = "tcp://*:5555";

/** \brief Estimate the hidden state of a system given a physical model and occasional measurements.
 * 
 * Kalman filter components:
 * A: Transition matrix: Defines how the state will evolve over time given no input and independent
 * of the control variable. For many systems, this is effectively dt. For constant systems, this is
 * identity.
 * B: Control matrix: Defines how the system evolves in response to the control variable.
 * H: Observation matrix: Translates the measured values into the state observation. If the state is
 * measured directly, this is the identity. This matrix will be something other than identity if the
 * state is measured indirectly.
 * P: Process covariance: Uncertainty in the state computed by the process. Grows over time while
 * measurements are not supplied, and shrinks when a measurement is supplied.
 * Q: Initial process covariance: Prevents process covariance reaching zero which would correspond
 * to complete trust in the process, ignoring measurements.
 * R: Measurement covariance: Uncertainty in measured values due to measurement noise.
 */
class VelocityKalmanFilter
{
    private:
        uint64_t last_time = 0;
        Eigen::Matrix<double, 3, 3> kalman_gain;                // K
        Eigen::Matrix<double, 3, 1> state{0, 0, 0};             // X
        Eigen::Matrix<double, 3, 3> state_transition;           // A
        Eigen::Matrix<double, 3, 3> control_matrix;             // B
        Eigen::Matrix<double, 3, 1> gravity{0, 0, 9.81};        // Gravity
        Eigen::Matrix<double, 3, 3> process_covariance;         // P
        Eigen::Matrix<double, 3, 3> measurement_covariance;     // R
        Eigen::Matrix<double, 3, 3> measurement_covariance_inv; // R.inv, for Mahalonobi distance
        Eigen::Matrix<double, 3, 3> initial_process_covariance; // Q;

        // Fitting matrices
        Eigen::Matrix<double, 3, 3> I{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        Eigen::Matrix<double, 3, 3> observation_matrix;         // H
        Eigen::Matrix<double, 3, 3> observation_matrix_tr;      // H.inv

        #ifdef DEBUGKALMAN
        arwain::Logger log;
        #endif

    public:
        VelocityKalmanFilter(double process_uncertainty, double measurement_uncertainty)
        {
            // The state is initialized at zero and is assumed constant without control input.
            state = Eigen::Matrix<double, 3, 1>{0, 0, 0};
            state_transition = I;

            auto& p = process_uncertainty;
            process_covariance = Eigen::Matrix<double, 3, 3>{
                {p*p,   0,   0},
                {  0, p*p,   0},
                {  0,   0, p*p}
            };
            initial_process_covariance = process_covariance;
            auto& m = measurement_uncertainty;
            measurement_covariance = Eigen::Matrix<double, 3, 3>{
                {m*m,   0,   0},
                {  0, m*m,   0},
                {  0,   0, m*m}
            };
            measurement_covariance_inv = measurement_covariance.inverse();
            observation_matrix = I;
            observation_matrix_tr = I;

            #ifdef DEBUGKALMAN
            log.open("kalman_log.txt");
            #endif
        }

        /** \brief Computes a distance metric between the measured value and the current state. The distance
         * is a generalization of standard deviation to multiple dimensions.
         */
        double mahalanobi_distance(const Eigen::Matrix<double, 3, 1>& measurement)
        {
            return std::sqrt((measurement - state).transpose() * measurement_covariance_inv * (measurement - state));
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

            control_matrix = Eigen::Matrix<double, 3, 3>{{dt, 0, 0},{0, dt, 0},{0, 0, dt}};

            #ifdef DEBUGKALMAN
            log << "State pre-update:\n";
            log << state << "\n";
            #endif

            Eigen::Matrix<double, 3, 1> acceleration{average_acceleration.x, average_acceleration.y, average_acceleration.z};
            
            /* The prediction step updates the state according to the model and measurements from the accelerometer.
                X = A*X + B*u + w
            */
            state = state_transition * state + control_matrix * (acceleration - gravity);

            #ifdef DEBUGKALMAN
            log << "State after state transition:" << "\n";
            log << state << "\n";
            #endif

            #ifdef DEBUGKALMAN
            log << "Process covariance pre-update 1:" << "\n";
            log << process_covariance << "\n";
            #endif

            /* The process covariance grows over time while measurements are not supplied.
                P = A*P*A.T + Q
            */
            process_covariance = state_transition * process_covariance * observation_matrix_tr + initial_process_covariance;

            #ifdef DEBUGKALMAN
            log << "Process covariance post-update 1:" << "\n";
            log << process_covariance << "\n";
            #endif

            #ifdef DEBUGKALMAN
            log << "Kalman gain pre-update:" << "\n";
            log << kalman_gain << "\n";
            #endif

            /* Update the Kalman gain based on the new process covariance.
                K = P*H * (H*P*H.T + R)^{-1}
            */
            kalman_gain = (process_covariance * observation_matrix) * (observation_matrix * process_covariance * observation_matrix_tr + measurement_covariance).inverse();

            #ifdef DEBUGKALMAN
            log << "Kalman gain post-update:" << "\n";
            log << kalman_gain << "\n";
            #endif

            /* Transform the measurement into state space.
                z = H*Y + omega
            */
            Eigen::Matrix<double, 3, 1> measurement = Eigen::Matrix<double, 3, 1>{velocity_measurement.x, velocity_measurement.y, velocity_measurement.z};
            #ifdef DEBUGKALMAN
            double dm = mahalanobi_distance(measurement);
            std::cout << "Dm(x): " << dm << std::endl;
            log << "Dm(x): " << dm << "\n";
            #endif

            /* Update the state using the measurement residual and the Kalman gain.
                X = X + K*(Y - X)
            */
            state = state + kalman_gain * (measurement - observation_matrix * state);

            #ifdef DEBUGKALMAN
            log << "State post measurement update:" << "\n";
            log << state << "\n";
            #endif

            /* Modify the process covariance according to the Kalman gain.
                P = (I - H*K) * P
            */
            process_covariance = (I - observation_matrix * kalman_gain) * process_covariance;

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

#if USENCS2
/** \brief Predicts velocity by passing IMU data to a neural compute resource, and integrates
 * velocity into position.
 */
void predict_velocity()
{
    if (arwain::config.no_inference)
    {
        return;
    }
    // *****************************************************************//
    // NOTE This relies upon a socket connection to a Python script     //
    /// which operates the NCS2, and should be deprecated and replaced. //
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

    uint64_t last_inference_time = 0;

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::Inference:
            {
                Vector3 position{0, 0, 0};
                Vector3 kalman_position{0, 0, 0};
                Vector3 velocity{0, 0, 0};
                
                VelocityKalmanFilter kalman{1.0, 0.0001}; // process, measurement

                // Open files for logging.
                // File handles for logging.
                arwain::Logger velocity_file;
                arwain::Logger kalman_velocity_file;
                arwain::Logger position_file;
                arwain::Logger kalman_position_file;

                velocity_file.open(arwain::folder_date_string + "/velocity.txt");
                kalman_velocity_file.open(arwain::folder_date_string + "/kalman_velocity.txt");
                position_file.open(arwain::folder_date_string + "/position.txt");
                kalman_position_file.open(arwain::folder_date_string + "/kalman_position.txt");
                velocity_file << "time x y z" << "\n";
                kalman_velocity_file << "time x y z" << "\n";
                position_file << "time x y z" << "\n";
                kalman_position_file << "time x y z" << "\n";

                // Set up timing.
                std::chrono::time_point<std::chrono::system_clock> lastTime = std::chrono::system_clock::now();
                std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now();
                std::chrono::milliseconds interval{arwain::Intervals::VELOCITY_PREDICTION_INTERVAL};

                while (arwain::system_mode == arwain::OperatingMode::Inference)
                {
                    // Reset the current position to zero if flag requests it.
                    if (arwain::reset_position)
                    {
                        position = {0, 0, 0};
                        kalman_position = {0, 0, 0};
                        arwain::reset_position = false;
                    }

                    imu = arwain::Buffers::IMU_WORLD_BUFFER.get_data();

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

                    // TODO Temporary to suppress errors in vertical velocity inference;
                    // Remove when root issue resolved.
                    if (std::abs(velocity.z) > 4.0)
                    {
                        velocity.z = 0.0;
                    }

                    // Feed the activity metrix.
                    arwain::activity_metric.feed_velo(velocity);

                    Vector3 average_acceleration{0, 0, 0};
                    for (std::deque<Vector6>::iterator it = imu.end() - 10; it != imu.end(); ++it)
                    {
                        average_acceleration = average_acceleration + (*it).acce;
                    }
                    average_acceleration = average_acceleration / 10.0;

                    Vector3 kalman_velocity = kalman.update(velocity, average_acceleration);

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
                    kalman_position = kalman_position + dt * kalman_velocity;

                    arwain::Buffers::POSITION_BUFFER.push_back(position);

                    // Log results to file.
                    velocity_file << time.time_since_epoch().count() << " " << velocity.x << " " << velocity.y << " " << velocity.z << "\n";
                    kalman_velocity_file << time.time_since_epoch().count() << " " << kalman_velocity.x << " " << kalman_velocity.y << " " << kalman_velocity.z << "\n";
                    position_file << time.time_since_epoch().count() << " " << position.x << " " << position.y << " " << position.z << "\n";
                    kalman_position_file << time.time_since_epoch().count() << " " << kalman_position.x << " " << kalman_position.y << " " << kalman_position.z << "\n";

                    // Attempt to keep track of what rate the NCS2 is operating at; it sometimes drops to 7 Hz due to thermal throttling,
                    // and this can affect other systems if it is not considered.
                    if (time.time_since_epoch().count() - last_inference_time > 100000000)
                    {
                        arwain::velocity_inference_rate = 7;
                    }
                    else
                    {
                        arwain::velocity_inference_rate = 20;
                    }

                    last_inference_time = time.time_since_epoch().count();

                    // Wait until next tick.
                    time = time + interval;
                    std::this_thread::sleep_until(time);
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
#else // Using tflite inference.
/** \brief Predicts velocity by passing IMU data to tensorflow-lite API. Integrates velocity into position. */
void predict_velocity()
{
    if (arwain::config.no_inference)
    {
        return;
    }

    // Wait for enough time to ensure the IMU buffer contains valid and useful data before starting.
    std::chrono::milliseconds presleep(1000);
    std::this_thread::sleep_for(presleep * 10);

    // Buffer to contain local copy of IMU data.
    std::deque<Vector6> imu;

    arwain::ready_for_inference = true;

    uint64_t last_inference_time = 0;

    // TODO Create inferencer.
    // If the model file cannot be found or loaded, set mode to terminate and break loop.
    std::unique_ptr<tflite::FlatBufferModel> model = tflite::FlatBufferModel::BuildFromFile(arwain::config.inference_model_xml.c_str());
    if (!model)
    {
        std::cout << "Failed to map model\n";
        arwain::system_mode = arwain::OperatingMode::Terminate;
        throw std::exception{};
    }
    // Configure interpreter.
    tflite::ops::builtin::BuiltinOpResolver resolver;
    std::unique_ptr<tflite::Interpreter> interpreter;
    tflite::InterpreterBuilder(*model.get(), resolver)(&interpreter);
    interpreter->AllocateTensors();
    float* input = interpreter->typed_input_tensor<float>(0);

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::Inference:
            {
                Vector3 position{0, 0, 0};
                Vector3 kalman_position{0, 0, 0};
                Vector3 velocity{0, 0, 0};
                VelocityKalmanFilter kalman{1.0, 0.0001};

                arwain::Logger velocity_file;
                arwain::Logger kalman_velocity_file;
                arwain::Logger position_file;
                arwain::Logger kalman_position_file;

                velocity_file.open(arwain::folder_date_string + "/velocity.txt");
                kalman_velocity_file.open(arwain::folder_date_string + "/kalman_velocity.txt");
                position_file.open(arwain::folder_date_string + "/position.txt");
                kalman_position_file.open(arwain::folder_date_string + "/kalman_position.txt");
                velocity_file << "time x y z" << "\n";
                kalman_velocity_file << "time x y z" << "\n";
                position_file << "time x y z" << "\n";
                kalman_position_file << "time x y z" << "\n";

                std::chrono::time_point<std::chrono::system_clock> last_time = std::chrono::system_clock::now();
                std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now();
                std::chrono::milliseconds interval{arwain::Intervals::VELOCITY_PREDICTION_INTERVAL};

                while (arwain::system_mode == arwain::OperatingMode::Inference)
                {
                    // Reset the current position to zero if flag requests it.
                    if (arwain::reset_position)
                    {
                        position = {0, 0, 0};
                        kalman_position = {0, 0, 0};
                        arwain::reset_position = false;
                    }

                    { // Grab latest IMU packet into local buffer.
                        std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
                        imu = arwain::Buffers::IMU_WORLD_BUFFER;
                    }

                    // Refresh time.
                    time = std::chrono::system_clock::now();

                    // Get dt in seconds since last update, and update last_time.
                    double dt = std::chrono::duration_cast<std::chrono::milliseconds>(time - last_time).count() / 1000.0;
                    last_time = time;

                    // Load data into inference and run inference.
                    unsigned int input_index = 0;
                    for (int i = 0; i < 6; i++)
                    {
                        for (int j = 0; j < 200; j++)
                        {
                            *(input + input_index) = imu[j][i];
                            input_index++;
                        }
                    }

                    interpreter->Invoke();
                    float* output = interpreter->typed_output_tensor<float>(0);
                    velocity = {output[0], output[1], output[2]};

                    // TODO Temporary to suppress large vertical errors in velocity inference.
                    // TODO Remove when root issue resolved.
                    if (std::abs(velocity.z) > 4.0)
                    {
                        velocity.z = 0.0;
                    }

                    // Feed the activity metric.
                    arwain::activity_metric.feed_velo(velocity);

                    Vector3 average_acceleration{0, 0, 0};
                    for (std::deque<Vector6>::iterator it = imu.end() -10; it != imu.end(); ++it)
                    {
                        average_acceleration = average_acceleration + (*it).acce;
                    }
                    average_acceleration = average_acceleration / 10.0;

                    Vector3 kalman_velocity = kalman.update(velocity, average_acceleration);

                    arwain::Buffers::VELOCITY_BUFFER.push_back(velocity);

                    // Attempt yaw drift velocity compensation if enabled.
                    if (arwain::config.correct_with_yaw_diff)
                    {
                        velocity = {
                            std::cos(-arwain::yaw_offset)*velocity.x - std::sin(-arwain::yaw_offset)*velocity.y,
                            std::sin(-arwain::yaw_offset)*velocity.x + std::cos(-arwain::yaw_offset)*velocity.y,
                            velocity.z
                        };
                    }

                    // Compute new position.
                    position = position + dt * velocity;
                    kalman_position = kalman_position + dt * kalman_velocity;

                    arwain::Buffers::POSITION_BUFFER.push_back(position);

                    // Log results to file.
                    velocity_file << time.time_since_epoch().count() << " " << velocity.x << " " << velocity.y << " " << velocity.z << "\n";
                    kalman_velocity_file << time.time_since_epoch().count() << " " << kalman_velocity.x << " " << kalman_velocity.y << " " << kalman_velocity.z << "\n";
                    position_file << time.time_since_epoch().count() << " " << position.x << " " << position.y << " " << position.z << "\n";
                    kalman_position_file << time.time_since_epoch().count() << " " << kalman_position.x << " " << kalman_position.y << " " << kalman_position.z << "\n";

                    last_inference_time = time.time_since_epoch().count();

                    time = time + interval;
                    std::this_thread::sleep_until(time);
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
}
#endif
