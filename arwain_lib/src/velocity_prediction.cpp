#include <iomanip>
#include <thread>
#include <cstring>
#include <eigen3/Eigen/Dense>

#include <arwain/event_manager.hpp> 

#include "arwain/velocity_prediction.hpp"
#include "arwain/logger.hpp"
#include "arwain/arwain.hpp"
#include "arwain/exceptions.hpp"
#include "arwain/thread.hpp"

PositionVelocityInference::PositionVelocityInference()
{
    init();
}

void PositionVelocityInference::core_setup()
{
    // Wait for enough time to ensure the IMU buffer contains valid and useful data before starting.
    std::this_thread::sleep_for(std::chrono::milliseconds{1000});
    
    // TODO Create inferencer.
    // If the model file cannot be found or loaded, set mode to terminate and break loop.
    model = tflite::FlatBufferModel::BuildFromFile(arwain::config.inference_model_path.c_str());
    if (!model)
    {
        std::cout << "Failed to map model\n";
        throw std::exception{};
    }
    // Configure interpreter.
    tflite::InterpreterBuilder(*model.get(), resolver)(&interpreter);
    interpreter->AllocateTensors();
    input = interpreter->typed_input_tensor<float>(0);
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
    position = {0, 0, 0};
    velocity = {0, 0, 0};
    velocity_file.open(arwain::folder_date_string + "/velocity.txt");
    position_file.open(arwain::folder_date_string + "/position.txt");
    velocity_file << "time x y z" << "\n";
    position_file << "time x y z" << "\n";
}

void PositionVelocityInference::run_inference()
{
    setup_inference();

    // Set up timing.
    // TODO Use JobInterval if possible; the timing here is different to other threads.
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> time = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds interval{arwain::Intervals::VELOCITY_PREDICTION_INTERVAL};

    while (mode == arwain::OperatingMode::Inference)
    {
        imu = arwain::Buffers::IMU_WORLD_BUFFER.get_data();
        // Check what the time really is since it might not be accurate.
        time = std::chrono::high_resolution_clock::now();
        // Get dt in seconds since last udpate, and update lastTime.
        double dt = std::chrono::duration_cast<std::chrono::milliseconds>(time - last_time).count()/1000.0;
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

        // TODO Temporary to suppress errors in vertical velocity inference;
        // Remove when root issue resolved.
        if (std::abs(velocity.z) > 4.0)
        {
            velocity.z = 0.0;
        }
        // Feed the activity metrix.
        arwain::activity_metric.feed_velocity(velocity);

        Vector3 average_acceleration{0, 0, 0};
        for (std::deque<ImuData>::iterator it = imu.end() - 10; it != imu.end(); ++it)
        {
            average_acceleration = average_acceleration + (*it).acce;
        }
        average_acceleration = average_acceleration / 10.0;

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

void PositionVelocityInference::cleanup_inference()
{
    position = {0, 0, 0};
    velocity = {0, 0, 0};
    position_file.close();
    velocity_file.close();
}

bool PositionVelocityInference::init()
{
    core_setup();
    if (job_thread.joinable())
    {
        job_thread.join();
    }
    job_thread = ArwainThread{&PositionVelocityInference::run, "arwain_infr_th", this};
    return true;
}

bool PositionVelocityInference::join()
{
    if (job_thread.joinable())
    {
        job_thread.join();
        return true;
    }
    return false;

    delete input;
}

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
