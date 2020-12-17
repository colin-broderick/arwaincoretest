#include <csignal>
#include <iostream>
#include <sstream>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>
#include <deque>
#include <stdint.h>
#include <array>
#include <math.h>
#include <vector>
#include <thread>
#include <mutex>
#include <fstream>

#include "quaternions.h"
#include "stance.h"
#include "pi_utils.h"
#include "madgwick.h"
#include "math_util.h"
#include "utils.h"

// #define LOGGING
#define LOGGING2

// Status codes.
unsigned int STATUS_FALLING = 1;
unsigned int STATUS_ENTANGLED = 2;
unsigned int STATUS_INACTIVE = 1 << 2;
unsigned int STATUS_WALKING = 2 << 2;
unsigned int STATUS_SEARCHING = 3 << 2;
unsigned int STATUS_CRAWLING = 4 << 2;
unsigned int STATUS_RUNNING = 5 << 2;
unsigned int STATUS_UNKNOWN_STANCE = 6 << 2;

// Characteristics of the sliding window for data fed to the NPU.
unsigned int STEP_SIZE = 50;
unsigned int WINDOW_SIZE = 200;

// Time intervals, all in milliseconds.
unsigned int IMU_READING_INTERVAL = 5;
unsigned int VELOCITY_PREDICTION_INTERVAL = 50;
unsigned int LORA_TRANSMISSION_INTERVAL = 1000;
unsigned int ALERT_SYSTEM_INTERVAL = 1000;

// Buffer sizes.
unsigned int POSITION_BUFFER_LEN = 200;
unsigned int VELOCITY_BUFFER_LEN = 200;
unsigned int ORIENTATION_BUFFER_LEN = 200;
unsigned int IMU_BUFFER_LEN = 200;

// IMU data buffers.
std::deque<std::array<float, 6>> imu_full_buffer;
std::deque<std::array<float, 6>> imu_stepped_buffer;
std::deque<std::array<float, 3>> vel_full_buffer;
std::deque<std::array<float, 3>> position_buffer;
std::deque<euler_orientation_t> euler_orientation_buffer;
std::deque<quat_orientation_t> quat_orientation_buffer;

extern std::string stance;
int status_flags = 0;

void efaroe_step(std::array<float, 3> gyro);

// Kill flag for all threads.
int shutdown = 0;

// Mutex locks.
std::mutex imu_buffer_lock;
std::mutex stepped_buffer_lock;
std::mutex vel_buffer_lock;
std::mutex status_flag_lock;
std::mutex position_buffer_lock;
extern std::mutex stance_lock;
std::mutex orientation_buffer_lock;

void std_output()
{
    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(1000);

    std::stringstream ss;

    while (!shutdown)
    {
        // Add position to the string stream.        
        position_buffer_lock.lock();
        ss << "Position:        (" << position_buffer.back()[0] << ", " << position_buffer.back()[1] << ", " << position_buffer.back()[2] << ")" << std::endl;
        position_buffer_lock.unlock();

        // Add Euler and quaternion orientations to the string stream.
        orientation_buffer_lock.lock();
        ss << "Orientation (E): (" << euler_orientation_buffer.back().roll << ", " << euler_orientation_buffer.back().pitch << ", " << euler_orientation_buffer.back().yaw << ")" << std::endl;;
        ss << "Orientation (Q): (" << quat_orientation_buffer.back().w << ", " << quat_orientation_buffer.back().x << ", " << quat_orientation_buffer.back().y << ", " << quat_orientation_buffer.back().z << ")" << std::endl;
        orientation_buffer_lock.unlock();

        // Add stance to the string stream.
        stance_lock.lock();
        ss << "Stance:          " << stance << std::endl;
        stance_lock.unlock();

        // TODO: Add fall/entanglement to the string stream.


        // Print the string.
        std::cout << ss.str();

        // Clear the stringstream.
        ss.str("");
        ss << std::endl;

        // Wait until next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }
}


/// Reads the IMU at (1000/IMU_READING_INTERVAL) Hz, and runs orientation updates at the same frequency.
void bmi270_reader()
{
    // Initialize the IMU.
    std::string path = "../calib.txt";
    if (init_bmi270(0, path) != 0)
    {
        printf("Node failed to start\n");
        exit(1);
    }

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(5);
    int count = 0;

    Madgwick orientation_filter;
    orientation_filter.begin(1000/IMU_READING_INTERVAL);

    // Local buffers for IMU data
    vec_scaled_output accel_data;
    vec_scaled_output gyro_data;
    euler_orientation_t euler_data;
    quat_orientation_t quat_data;

    #ifdef LOGGING
    // Open file handles for data logging.
    std::ofstream acce_file("../acce.txt");
    std::ofstream gyro_file("../gyro.txt");
    std::ofstream euler_file("../euler_orientation.txt");
    std::ofstream quat_file("../quat_orientation.txt");
    std::ofstream mag_file("../mag.txt");

    // File headers
    acce_file << "# time x y z" << std::endl;
    gyro_file << "# time x y z" << std::endl;
    mag_file << "# time x y z" << std::endl;
    euler_file << "# time roll pitch roll" << std::endl;
    quat_file << "# time w x y z" << std::endl;
    #endif

    while (!shutdown)
    {
        // Read current sensor values.
        get_bmi270_data(&accel_data, &gyro_data);

        // Add new reading to end of buffer, and remove oldest reading from start of buffer.
        imu_buffer_lock.lock();
        imu_full_buffer.push_back(std::array<float, 6>{
            accel_data.x, accel_data.y, accel_data.z,
            gyro_data.x, gyro_data.y, gyro_data.z
        });
        imu_full_buffer.pop_front();
        imu_buffer_lock.unlock();

        #ifdef LOGGING
        // Log IMU to file.
        acce_file << time << " " << accel_data.x << " " << accel_data.y << " " << accel_data.z << std::endl;
        gyro_file << time << " " << gyro_data.x << " " << gyro_data.y << " " << gyro_data.z << std::endl;
        #endif

        // Perform a Madgwick step
        orientation_filter.updateIMU(
            gyro_data.x,
            gyro_data.y,
            gyro_data.z,
            accel_data.x,
            accel_data.y,
            accel_data.z
        );

        // Extract Euler orientation from filter.
        euler_data.roll = orientation_filter.getRoll();
        euler_data.pitch = orientation_filter.getPitch();
        euler_data.yaw = orientation_filter.getYaw();

        // Extract quaternion orientation from filter.
        quat_data.w = orientation_filter.getW();
        quat_data.x = orientation_filter.getX();
        quat_data.y = orientation_filter.getY();
        quat_data.z = orientation_filter.getZ();

        // Add orientation information to buffers.
        orientation_buffer_lock.lock();
        euler_orientation_buffer.push_back(euler_data);
        euler_orientation_buffer.pop_front();
        quat_orientation_buffer.push_back(quat_data);
        quat_orientation_buffer.pop_front();
        orientation_buffer_lock.unlock();

        #ifdef LOGGING
        // Log orientation information to file.
        euler_file << time << " " << euler_data.roll << " " << euler_data.pitch << " " << euler_data.yaw << std::endl;
        quat_file << time << " " << quat_data.w << " " << quat_data.x << " " << quat_data.y << " " << quat_data.z << std::endl;;
        #endif

        // Perform an efaroe step to update orientation.
        // efaroe_step(std::array<float, 3>{
        //     gyro_data.x,
        //     gyro_data.y,
        //     gyro_data.z
        // });
   
        // count++;
        // if (count % WINDOW_SIZE == 0)
        // {
        //     std::cout << (int)(count/WINDOW_SIZE) << std::endl;
        //     std::cout << orientation_filter.getRoll() << "," << orientation_filter.getPitch() << "," << orientation_filter.getYaw() << std::endl;
        // }
        
        // Wait until the next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    #ifdef LOGGING
    // Close all file handles.
    acce_file.close();
    gyro_file.close();
    euler_file.close();
    quat_file.close();
    mag_file.close();
    #endif
}


// class efaroe
// {
//     public:

//         std::array<float, 3> gyro_bias;
//         std::array<float, 3> emf;
//         float zeta;
//         int last_read;
//         int use_mag;
//         float gyro_error;
//         float true_error;
//         float beta;
//         quaternion q;
//         int conv_count;
//         float uk_dip = -67*3.14159/180;

//         efaroe(quaternion initial_quaternion, std::array<float, 3> gyro_bias, float gyro_error, int use_mag)
//         {
//             this->gyro_bias = gyro_bias;
//             this->emf = std::array<float, 3>{cos(uk_dip), 0, sin(uk_dip)};
//             this->zeta = sqrt(3) * 1e-2;
//             this->last_read = 0;
//             this->use_mag = use_mag;

//             this->q = initial_quaternion;
//             this->gyro_error = gyro_error;
//             this->true_error = gyro_error;
//             this->beta = sqrt(3) * this->gyro_error;
//             this->conv_count = -1;
//         }

//         void update(int reading_time, std::array<float, 6> imu)
//         {
//             if (conv_count >= 0)
//             {
//                 conv_count = conv_count -1;
//                 if (conv_count == 0)
//                 {
//                     gyro_error = true_error;
//                     beta = sqrt(3) * gyro_error;
//                 }
//             }

//             if (last_read == 0)
//             {
//                 last_read = reading_time;
//                 return;
//             }
//             else {
//                 float dt = reading_time - last_read;
//                 if (dt > 1)
//                 {
//                     dt = 1;
//                 }
//                 last_read = reading_time;
//             }

//             double acce_norm = norm(std::array<double, 3>{imu[0], imu[1], imu[2]});
//             std::array<double, 3> acce{
//                 imu[0] / acce_mag,
//                 imu[1] / acce_mag,
//                 imu[2] / acce_mag
//             };
//             std::array<double, 3> jac_a{
//                 q.getIm_i() * q.getIm_k() * 2 - q.getRe() * q.getIm_j() * 2, 
//                 q.getRe() * q.getIm_i() * 2 + q.getIm_j() * q.getIm_k() * 2,
//                 1 - (2 * q.getIm_i() * q.getIm_i()) - (2 * q.getIm_j() * q.getIm_j())
//             };

//             // Get normalized gradient.
//             std::array<double, 3> grad = cross(jac_a, acce);
//             double grad_norm = norm(grad);
//             grad = std::array<double, 3>{
//                 grad[0]/grad_norm,
//                 grad[1]/grad_norm,
//                 grad[2]/grad_norm
//             };

//             g_b = gyro_bias + (grad * dt * zeta);
//             gyro = gyro_raw - g_b;

//             a_v = 
//         }
// };


// void efaroe_step(std::array<float, 3> gyro)
// {

// }

/// TODO: I think this might be unneeded. Make sure, then remove.
/// Makes a copy of the full imu buffer after every nth time step.
void imu_stepper()
{
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(1000/WINDOW_SIZE * STEP_SIZE);
    
    while (!shutdown)
    {
        // Copy existing full buffer into stepped buffer
        imu_buffer_lock.lock();
        stepped_buffer_lock.lock();
        imu_stepped_buffer = imu_full_buffer;
        imu_buffer_lock.unlock();
        stepped_buffer_lock.unlock();

        // Wait until the next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }
}

/// Periodically sends the most recent position and status information via LoRa.
int transmit_lora()
{
    // TODO: Set up radio.

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(LORA_TRANSMISSION_INTERVAL);

    std::array<float, 3> position;
    unsigned int status;

    #ifdef LOGGING
    // Open file handles for data logging.
    std::ofstream lora_file("../lora_log.txt");
    lora_file << "# time packet" << std::endl;
    #endif

    while (!shutdown)
    {
        // std::cout << "LoRa tick\n" << std::endl;
        // TODO: Read all relevant data, respecting mutex locks.
        // Get positions as uint16
        position_buffer_lock.lock();
        position = position_buffer.back();
        position_buffer_lock.unlock();
        uint64_t x = (uint64_t)position[0];
        uint64_t y = (uint64_t)position[1];
        uint64_t z = (uint64_t)position[2];

        // Get current status flags.
        status_flag_lock.lock();
        status = (uint64_t)status_flags;
        status_flag_lock.unlock();

        // Build packet for transmission.
        uint64_t packet = (x << 48) | (y << 32) | (z << 16) | (status);
    
        // TODO: Send transmission.
        // std::cout << packet << std::endl;

        // TODO: Maybe a read loop?

        // TODO: Log LoRa transmission to file, including any success/signal criteria that might be available.
        #ifdef LOGGING
        lora_file << time << " " << packet << std::endl;
        #endif

        // Wait until next tick
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    #ifdef LOGGING
    // Close log file handle.
    lora_file.close();
    #endif

    return 1;
}

/// Sets up and runs velocity prediction using the NPU. Run as a thread.
int predict_velocity()
{
    // TODO: Set up NPU and feed in model.

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(VELOCITY_PREDICTION_INTERVAL);

    std::array<float, 3> vel;
    std::array<float, 3> pos;
    std::deque<std::array<float, 6>> imu;

    float interval_seconds = (float)VELOCITY_PREDICTION_INTERVAL/1000.0;

    #ifdef LOGGING
    // Open files for logging.
    std::ofstream velocity_file("../velocity.txt");
    std::ofstream position_file("../position.txt");
    velocity_file << "# time x y z" << std::endl;
    position_file << "# time x y z" << std::endl;
    #endif

    while (!shutdown)
    {
        // Grab latest IMU packet
        imu_buffer_lock.lock();
        imu = imu_full_buffer;
        imu_buffer_lock.unlock();

        // TODO World align latest IMU data; (IMU \op orientation = world-aligned-imu)

        // TODO: Make velocity prediction
        vel = std::array<float, 3>{0, 0, 0};

        // Add velocity to buffer.
        vel_buffer_lock.lock();
        vel_full_buffer.push_back(vel);
        vel_full_buffer.pop_front();
        vel_buffer_lock.unlock();

        // Iterate velocity onto position to get new position.
        pos[0] = pos[0] + interval_seconds * vel[0];
        pos[1] = pos[1] + interval_seconds * vel[1];
        pos[2] = pos[2] + interval_seconds * vel[2];
        
        // Update position buffer.
        position_buffer_lock.lock();
        position_buffer.push_back(pos);
        position_buffer.pop_front();
        position_buffer_lock.unlock();

        #ifdef LOGGING
        // Add position and velocity data to file.
        velocity_file << time << " " << vel[0] << " " << vel[1] << " " << vel[2] << std::endl;
        position_file << time << " " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
        #endif

        // Wait until next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    #ifdef LOGGING
    // Close file handle(s).
    velocity_file.close();
    position_file.close();
    #endif

    return 1;
}

/// Captures keyboard interrupt. Waits for threads to end then program exits.
void sigint_handler(int signal)
{
    std::cout << "\nReceived SIGINT - closing\n" << std::endl;
    shutdown = 1;
}

/// TODO This might be unnecessary; since the stuff it's setting up only matter to the LoRa thread,
/// it may as well be done there.
void alert_system()
{
    // todo setup

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(ALERT_SYSTEM_INTERVAL);

    while (!shutdown)
    {
        // Update status flags.
        status_flag_lock.lock();
        status_flags = 0;
        if (is_falling())
        {
            status_flags |= STATUS_FALLING;
        }
        if (is_entangled())
        {
            status_flags |= STATUS_ENTANGLED;
        }
        if (stance == "inactive")
        {
            status_flags |= STATUS_INACTIVE;
        }
        else if (stance == "walking")
        {
            status_flags |= STATUS_WALKING;
        }
        else if (stance == "crawling")
        {
            status_flags |= STATUS_CRAWLING;
        }
        else if (stance == "searching")
        {
            status_flags |= STATUS_SEARCHING;
        }
        else if (stance == "unknown")
        {
            status_flags |= STATUS_UNKNOWN_STANCE;
        }
        status_flag_lock.unlock();
        
        // Wait until next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }
}

/// This is a template for implementing new functionality.
void thread_template()
{
    // TODO DECLARE ANY LOCAL VARIABLES OUTSIDE THE WHILE LOOP.
    float var;
    float from_global;
    
    // TODO IF LOGGING, OPEN FILE HANDLES.
    #ifdef LOGGING2
    std::ofstream log_file("../log_file.txt");
    log_file << "# time value1" << std::endl;
    #endif

    // TODO CONFIGURE TIMING INTERVAL.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(1000);

    // TODO START LOOP, RESPECTING SHUTDOWN FLAG.
    while (!shutdown)
    {
        // TODO GLOBAL DATA INTO LOCAL BUFFERS BEFORE USE. RESPECT MUTEX LOCKS.
        position_buffer_lock.lock();
        from_global = 1.5;
        position_buffer_lock.unlock();

        // TODO UPDATE LOCAL VARS.
        var = 3.0 + from_global;

        // TODO IF LOGGING, UPDATE LOG FILE.
        #ifdef LOGGING2
        log_file << time << " " << var << std::endl;
        #endif

        // TODO ADD NEW VALUES TO GLOBAL BUFFER, RESPECTING MUTEX LOCKS.
        position_buffer_lock.lock();
        position_buffer.push_back(std::array<float, 3>{var, 0, 0});
        position_buffer_lock.unlock();

        // TODO WAIT UNTIL NEXT TIME INTERVAL.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    // TODO IF LOGGING, CLOSE FILE HANDLES.
    #ifdef LOGGING2
    log_file.close();
    #endif
}


/// mainloop
int main()
{
    // Prepare keyboard interrupt signal handler to enable graceful exit.
    std::signal(SIGINT, sigint_handler);

    // Preload buffers.
    for (unsigned int i = 0; i < IMU_BUFFER_LEN; i++)
    {
        imu_full_buffer.push_back(std::array<float, 6>{0, 0, 0, 0, 0, 0});
        imu_stepped_buffer.push_back(std::array<float, 6>{0, 0, 0, 0, 0, 0});
    }
    for (unsigned int i = 0; i < VELOCITY_BUFFER_LEN; i++)
    {
        vel_full_buffer.push_back(std::array<float, 3>{0, 0, 0});
    }
    for (unsigned int i = 0; i < POSITION_BUFFER_LEN; i++)
    {
        position_buffer.push_back(std::array<float, 3>{0, 0, 0});
    }
    for (unsigned int i = 0; i < ORIENTATION_BUFFER_LEN; i++)
    {
        euler_orientation_buffer.push_back(euler_orientation_t{0, 0, 0});
        quat_orientation_buffer.push_back(quat_orientation_t{0, 0, 0});
    }

    // Start threads
    std::thread imu_full_thread(bmi270_reader);
    std::thread imu_step_thread(imu_stepper);
    std::thread velocity_prediction(predict_velocity);
    std::thread fall_detection(run_fall_detect);
    std::thread stance_detection(run_stance_detect);
    std::thread radio_thread(transmit_lora);
    std::thread alert_thread(alert_system);
    std::thread logging_thread(std_output);

    // Spin until shutdown signal received.
    std::chrono::milliseconds delay(1000);
    while (!shutdown)
    {
        std::this_thread::sleep_for(delay);
    }

    // Wait for all threads to terminate.
    imu_full_thread.join();
    imu_step_thread.join();
    velocity_prediction.join();
    fall_detection.join();
    stance_detection.join();
    radio_thread.join();
    alert_thread.join();
    logging_thread.join();
}
