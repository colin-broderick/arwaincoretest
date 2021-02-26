#include <algorithm>
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
#include <experimental/filesystem>

#include "arwain_torch.h"
#include "quaternions.h"
#include "stance.h"
#include "imu_utils.h"
#include "madgwick.h"
#include "efaroe.h"
#include "math_util.h"
// #include "rknn_api.h"
#include "input_parser.h"
#include "bin_log.h"

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
unsigned int VELOCITY_PREDICTION_INTERVAL = 500;
unsigned int LORA_TRANSMISSION_INTERVAL = 1000;
unsigned int ALERT_SYSTEM_INTERVAL = 1000;

// Buffer sizes.
unsigned int POSITION_BUFFER_LEN = 200;
unsigned int MAG_BUFFER_LEN = 200;
unsigned int VELOCITY_BUFFER_LEN = 200;
unsigned int ORIENTATION_BUFFER_LEN = 200;
unsigned int IMU_BUFFER_LEN = 200;

// Create a struct to hold configuration information.
configuration CONFIG;

// Default config file locations.
std::string config_file = "./arwain.conf";

// Flags for whether to produce various log outputs.
int log_to_std = 0;
int log_to_file = 0;
int no_inf = 0;

// Name for data folder
std::string folder_date_string;

// IMU data buffers.
std::deque<std::array<double, 6>> imu_buffer;
std::deque<std::array<double, 6>> imu_world_buffer;
std::deque<std::array<double, 3>> vel_buffer;
std::deque<std::array<double, 3>> position_buffer;
std::deque<std::array<double, 3>> mag_buffer;
std::deque<std::array<double, 3>> mag_world_buffer;
std::deque<euler_orientation_t> euler_orientation_buffer;
std::deque<quaternion> quat_orientation_buffer;

// Stance flags.
std::string stance_;
int horizontal;
int entangled;
int falling;
int status_flags = 0;

// Kill flag for all threads.
int shutdown = 0;

// Mutex locks.
std::mutex imu_buffer_lock;
std::mutex mag_buffer_lock;
std::mutex vel_buffer_lock;
std::mutex status_flag_lock;
std::mutex position_buffer_lock;
std::mutex orientation_buffer_lock;
extern std::mutex stance_lock;

// Sensor biases.
vector3 gyro_bias;
vector3 accel_bias;
vector3 mag_bias;
vector3 mag_scale;

/** \brief Periodically logs status messages to stdout.
 */
void std_output()
{
    if (log_to_std)
    {
        // Output string built here.
        std::stringstream ss;
        
        // Local containers for stances statuses.
        // std::string fall;
        // std::string entd;
        // std::string ori;

        // Set up timing, including pause while IMU warms up.
        std::chrono::milliseconds interval(100);
        std::this_thread::sleep_for(interval*3);
        auto time = std::chrono::system_clock::now();

        while (!shutdown)
        {
            // Add position to the string stream.        
            position_buffer_lock.lock();
            ss << "Position:        (" << position_buffer.back()[0] << ", " << position_buffer.back()[1] << ", " << position_buffer.back()[2] << ")" << "\n";
            position_buffer_lock.unlock();

            // Add Euler and quaternion orientations to the string stream.
            orientation_buffer_lock.lock();
            ss << "Orientation (E): (" << euler_orientation_buffer.back().roll << ", " << euler_orientation_buffer.back().pitch << ", " << euler_orientation_buffer.back().yaw << ")" << "\n";;
            ss << "Orientation (Q): (" << quat_orientation_buffer.back().w << ", " << quat_orientation_buffer.back().x << ", " << quat_orientation_buffer.back().y << ", " << quat_orientation_buffer.back().z << ")" << "\n";
            orientation_buffer_lock.unlock();

            // Add stance to the string stream.
            // fall = is_falling() ? "ON" : "OFF";
            // entd = is_entangled() ? "ON" : "OFF";
            // ori = is_horizontal() ? "horizontal" : "vertical";
            ss << "Stance:          " << stance_ << ", " <<  horizontal << "\n";
            ss << "Fall flag:       " << falling << "\n";
            ss << "Entangled flag:  " << entangled << "\n";

            // Print the string.
            std::cout << ss.str() << std::endl;

            // Clear the stringstream.
            ss.str("");

            // Wait until next tick.
            time = time + interval;
            std::this_thread::sleep_until(time);
        }
    }
}

/** \brief Reads IMU data, runs orientation filter(s), rotates IMU data, and buffers everything.
 */
void imu_reader()
{
    // Initialize orientation filter.
    // Madgwick orientation_filter{1000.0/IMU_READING_INTERVAL, CONFIG.madgwick_beta};
    eFaroe orientation_filter{
        quaternion{0,0,0,0},
        gyro_bias,
        100,
        0
    };

    int count = 0;
    int get_mag = 0;

    // Local buffers for IMU data.
    vector3 accel_data;
    vector3 gyro_data;
    vector3 mag_data;
    vector3 world_accel_data;
    vector3 world_gyro_data;
    // vector3 world_mag_data;
    euler_orientation_t euler_data;
    quaternion quat_data;

    // File handles for logging.
    std::ofstream acce_file;
    std::ofstream gyro_file;
    std::ofstream mag_file;
    std::ofstream euler_file;
    std::ofstream quat_file;

    if (log_to_file)
    {
        // Open file handles for data logging.
        acce_file.open(folder_date_string + "/acce.txt");
        gyro_file.open(folder_date_string + "/gyro.txt");
        mag_file.open(folder_date_string + "/mag.txt");
        euler_file.open(folder_date_string + "/euler_orientation.txt");
        quat_file.open(folder_date_string + "/quat_orientation.txt");

        // File headers
        acce_file << "# time x y z" << "\n";
        gyro_file << "# time x y z" << "\n";
        mag_file << "# time x y z" << "\n";
        euler_file << "# time roll pitch yaw" << "\n";
        quat_file << "# time w x y z" << "\n";
    }

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(5);

    while (!shutdown)
    {
        // Whether or not to get magnetometer on this spin.
        get_mag = ((count % 20 == 0) && (CONFIG.use_magnetometer || CONFIG.log_magnetometer));

        // Read current sensor values and apply bias correction.
        get_bmi270_data(&accel_data, &gyro_data);
        if (get_mag)
        {
            get_bmm150_data(&mag_data);
        }
        accel_data = accel_data - accel_bias;
        gyro_data = gyro_data - gyro_bias;
        mag_data = mag_data - mag_bias;
        mag_data = mag_data * mag_scale;
        
        // Add new reading to end of buffer, and remove oldest reading from start of buffer.
        imu_buffer_lock.lock();
        imu_buffer.push_back(std::array<double, 6>{
            accel_data.x, accel_data.y, accel_data.z,
            gyro_data.x, gyro_data.y, gyro_data.z
        });
        imu_buffer.pop_front();
        imu_buffer_lock.unlock();

        // Buffer mag_data if collected. TODO Is there actually any reason to buffer this?
        // if (get_mag)
        // {
        //     mag_buffer_lock.lock();
        //     mag_buffer.push_back(std::array<double, 3>{mag_data.x, mag_data.y, mag_data.z});
        //     mag_buffer.pop_front();
        //     mag_buffer_lock.unlock();
        // }

        // Log IMU to file.
        if (log_to_file)
        {
            acce_file << time.time_since_epoch().count() << " " << accel_data.x << " " << accel_data.y << " " << accel_data.z << "\n";
            gyro_file << time.time_since_epoch().count() << " " << gyro_data.x << " " << gyro_data.y << " " << gyro_data.z << "\n";
            if (CONFIG.log_magnetometer)
            {
                mag_file << time.time_since_epoch().count() << " " << mag_data.x << " " << mag_data.y << " " << mag_data.z << "\n";
            }
        }

        // Perform an orientation filter step.
        if (CONFIG.use_magnetometer)
        {
            orientation_filter.update(
                time.time_since_epoch().count(),
                gyro_data.x, gyro_data.y, gyro_data.z,
                accel_data.x, accel_data.y, accel_data.z,
                mag_data.x, mag_data.y, mag_data.z
            );
        }
        else
        {
            orientation_filter.update(
                time.time_since_epoch().count(),
                gyro_data.x, gyro_data.y, gyro_data.z,
                accel_data.x, accel_data.y, accel_data.z
            );
        }

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

        // Add world-aligned IMU to its own buffer.
        world_accel_data = world_align(accel_data, quat_data);
        world_gyro_data = world_align(gyro_data, quat_data);
        imu_buffer_lock.lock();
        imu_world_buffer.push_back(std::array<double, 6>{
            world_accel_data.x, world_accel_data.y, world_accel_data.z,
            world_gyro_data.x, world_gyro_data.y, world_gyro_data.z
        });
        imu_world_buffer.pop_front();
        imu_buffer_lock.unlock();

        // Add world-aligned magnetic field to buffer. TODO Why?
        // if (get_mag)
        // {
        //     world_mag_data = world_align(mag_data, quat_data);
        //     mag_buffer_lock.lock();
        //     mag_world_buffer.push_back(std::array<double, 3>{
        //         world_mag_data.x, world_mag_data.y, world_mag_data.z
        //     });
        //     mag_world_buffer.pop_front();
        //     mag_buffer_lock.unlock();
        // }

        // Log orientation information to file.
        if (log_to_file)
        {
            euler_file << time.time_since_epoch().count() << " " << euler_data.roll << " " << euler_data.pitch << " " << euler_data.yaw << "\n";
            quat_file << time.time_since_epoch().count() << " " << quat_data.w << " " << quat_data.x << " " << quat_data.y << " " << quat_data.z << "\n";
        }
        
        // Wait until the next tick.
        count++;
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    // Close all file handles.
    if (log_to_file)
    {
        acce_file.close();
        gyro_file.close();
        euler_file.close();
        quat_file.close();
        mag_file.close();
    }
}

/** \brief Forms and transmits LoRa messages on a loop.
 */
int transmit_lora()
{
    // TODO: Set up radio.

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(LORA_TRANSMISSION_INTERVAL);

    std::ofstream lora_file;

    std::array<double, 3> position;
    unsigned int status;

    // Open file handles for data logging.
    if (log_to_file)
    {
        lora_file.open(folder_date_string + "/lora_log.txt");
        lora_file << "# time packet" << "\n";
    }

    while (!shutdown)
    {
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
        // std::cout << packet << "\n";

        // TODO: Maybe a read loop?

        // TODO: Log LoRa transmission to file, including any success/signal criteria that might be available.
        if (log_to_file)
        {
            lora_file << time.time_since_epoch().count() << " " << packet << "\n";
        }
        
        // Wait until next tick
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    // Close log file handle.
    if (log_to_file)
    {
        lora_file.close();
    }

    return 1;
}

// Overload array operator*.
template <class T, class U>
T operator*(const T& a1, U scalar)
{
  T a;
  for (typename T::size_type i = 0; i < a1.size(); i++)
    a[i] = a1[i]*scalar;
  return a;
}

// Overload array operator-.
template <class T>
T operator-(const T& a1, const T& a2)
{
  T a;
  for (typename T::size_type i = 0; i < a1.size(); i++)
    a[i] = a1[i] - a2[i];
  return a;
}

// Overload array operator+.
template <class T>
T operator+(const T& a1, const T& a2)
{
  T a;
  for (typename T::size_type i = 0; i < a1.size(); i++)
    a[i] = a1[i] + a2[i];
  return a;
}

/** \brief Integrates acceleration data over a small window to generate velocity delta.
 * \param data The buffer to integrate over.
 * \param dt The time delta between buffer values.
 * \param offset The position in the buffer entries of the first acceleration value.
 * \return An array of x, y, z velocities.
 */
std::array<double, 3> integrate(std::deque<std::array<double, 6>> data, double dt, unsigned int offset = 0)
{
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

/** \brief Converts the IMU deque into a C-style array for passing to the inference library.
 * \param[out] data C-style array of shape [1][6][200] in which to store result.
 * \param[in] imu The latest world IMU buffer.
 */
void torch_array_from_deque(float data[1][6][200], std::deque<std::array<double, 6>> imu)
{
    for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 200; k++)
            {
                // Switch the order of gyro and acceleration, and put into array.
                data[i][j][k] = (float)(imu[k][(j+3)%6]);
            }
        }
    }
}

/** \brief Periodically makes velocity predictions based on data buffers, and adds that velocity and thereby position to the relevant buffers.
 */
void predict_velocity()
{
    // Skip inference if command line says so.
    if (no_inf)
    {
        return;
    }

    // TODO: Merge the inference code into this function. Will need further abstraction?
    // TODO: Set up NPU and feed in model.
    // Torch model{"./xyzronin_v0-5_all2D_small.pt", {1, 6, 200, 1}};
    Torch model{"./xyzronin_v0-6.pt", {1, 6, 200}};

    // Wait for enough time to ensure the IMU buffer contains valid and useful data before starting.
    std::chrono::milliseconds presleep(1000);
    std::this_thread::sleep_for(presleep*3);

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(VELOCITY_PREDICTION_INTERVAL);

    // Initialize buffers to contain working values.
    std::array<double, 3> vel;                    // The sum of npu_vel and imu_vel_delta.
    std::array<double, 3> npu_vel;                // To hold the neural network prediction of velocity.
    std::array<double, 3> vel_previous;           // Contains the velocity calculation from the previous loop.
    std::array<double, 3> npu_vel_delta;          // Stores the difference between the npu velocity prediction and vel_previous.
    std::array<double, 3> imu_vel_delta;          // To integrate the velocity based on IMU readings.
    std::array<double, 3> pos;
    std::deque<std::array<double, 6>> imu;        // To contain the last second of IMU data.
    std::deque<std::array<double, 6>> imu_latest; // To contain the last VELOCITY_PREDICTION_INTERVAL of IMU data.

    // File handles for logging.
    std::ofstream position_file;
    std::ofstream velocity_file;

    // Time in seconds between inferences.
    double interval_seconds = ((double)(VELOCITY_PREDICTION_INTERVAL))/1000.0;

    // TEST How far back to look in the IMU buffer for integration.
    int backtrack = (int)((1000/IMU_READING_INTERVAL)*interval_seconds);

    // Open files for logging.
    if (log_to_file)
    {
        velocity_file.open(folder_date_string + "/velocity.txt");
        position_file.open(folder_date_string + "/position.txt");
        velocity_file << "# time x y z" << "\n";
        position_file << "# time x y z" << "\n";
    }

    float data[1][6][200];
    
    while (!shutdown)
    {
        // Grab latest IMU packet
        imu_buffer_lock.lock();
        imu = imu_world_buffer;
        imu_buffer_lock.unlock();

        // TEST Create data array that torch can understand.
        torch_array_from_deque(data, imu);

        // TEST Make velocity prediction
        auto v = model.infer(data);
        npu_vel = {v[0], v[1], v[2]};
        
        // TEST Find the change in velocity from the last period, as predicted by the npu.
        npu_vel_delta = npu_vel - vel_previous;

        // TEST Get last interval worth of IMU data.
        imu_latest = {imu.end() - backtrack, imu.end()};

        // TEST Single integrate the small IMU slice to get delta-v over the period.
        imu_vel_delta = integrate(imu_latest, interval_seconds );

        // TEST Weighted combination of velocity deltas from NPU and IMU integration.
        vel = npu_vel_delta*CONFIG.npu_vel_weight_confidence + imu_vel_delta*(1-CONFIG.npu_vel_weight_confidence);

        // TEST Add the filtered delta onto the previous vel estimate and add to buffer.
        vel = vel + vel_previous;

        vel_buffer_lock.lock();
        vel_buffer.push_back(vel);
        vel_buffer.pop_front();
        vel_buffer_lock.unlock();

        // TEST Store the velocity for use in the next loop (saves having to access the buffer for a single element).
        vel_previous = vel;

        // Iterate velocity onto position to get new position.
        pos[0] = pos[0] + interval_seconds * vel[0];
        pos[1] = pos[1] + interval_seconds * vel[1];
        pos[2] = pos[2] + interval_seconds * vel[2];
        
        // Update position buffer.
        position_buffer_lock.lock();
        position_buffer.push_back(pos);
        position_buffer.pop_front();
        position_buffer_lock.unlock();

        // Add position and velocity data to file.
        if (log_to_file)
        {
            velocity_file << time.time_since_epoch().count() << " " << vel[0] << " " << vel[1] << " " << vel[2] << "\n";
            position_file << time.time_since_epoch().count() << " " << pos[0] << " " << pos[1] << " " << pos[2] << "\n";
        }

        // Wait until next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    // Close file handle(s).
    if (log_to_file)
    {
        velocity_file.close();
        position_file.close();
    }
}

/** \brief Capture the SIGINT signal for clean exit.
 * Sets the global shutdown flag informing all threads to clean up and exit.
 * \param signal The signal to capture.
 */
void sigint_handler(int signal)
{
    if (signal == SIGINT)
    {
        std::cout << "\nReceived SIGINT - closing\n" << "\n";
        shutdown = 1;
    }
}

/// TODO This might be unnecessary; since the stuff it's setting up only matters to the LoRa thread,
/// it may as well be done there.
// void alert_system()
// {
//     // todo setup

//     // Set up timing.
//     auto time = std::chrono::system_clock::now();
//     std::chrono::milliseconds interval(ALERT_SYSTEM_INTERVAL);

//     while (!shutdown)
//     {
//         // Update status flags.
//         status_flag_lock.lock();
//         status_flags = 0;
//         if (is_falling())
//         {
//             status_flags |= STATUS_FALLING;
//         }
//         if (is_entangled())
//         {
//             status_flags |= STATUS_ENTANGLED;
//         }
//         if (stance_ == "inactive")
//         {
//             status_flags |= STATUS_INACTIVE;
//         }
//         else if (stance_ == "walking")
//         {
//             status_flags |= STATUS_WALKING;
//         }
//         else if (stance_ == "crawling")
//         {
//             status_flags |= STATUS_CRAWLING;
//         }
//         else if (stance_ == "searching")
//         {
//             status_flags |= STATUS_SEARCHING;
//         }
//         else if (stance_ == "unknown")
//         {
//             status_flags |= STATUS_UNKNOWN_STANCE;
//         }
//         status_flag_lock.unlock();
        
//         // Wait until next tick.
//         time = time + interval;
//         std::this_thread::sleep_until(time);
//     }
// }

/// This function is never called. It should be considered a template
/// for implementing new threaded functionality. All existing threads
/// follow approximately this design pattern.
void thread_template()
{
    // DECLARE ANY LOCAL VARIABLES OUTSIDE THE WHILE LOOP.
    double var;
    double from_global;

    // IF LOGGING, CREATE FILE HANDLE
    // If using arwain binary logging (under development) just do
    // arwain::BinLog log_file("file_name.txt", arwain::accelwrite)
    std::ofstream log_file;

    // IF LOGGING, OPEN FILE HANDLES.
    if (log_to_file)
    {
        log_file.open(folder_date_string + "/log_file.txt");
        log_file << "# time value1" << "\n";
    }

    // CONFIGURE TIMING INTERVAL.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(1000);

    // START LOOP, RESPECTING SHUTDOWN FLAG.
    while (!shutdown)
    {
        // GLOBAL DATA INTO LOCAL BUFFERS BEFORE USE. RESPECT MUTEX LOCKS.
        position_buffer_lock.lock();
        from_global = 1.5;
        position_buffer_lock.unlock();

        // UPDATE LOCAL VARS.
        var = 3.0 + from_global;

        // IF LOGGING, UPDATE LOG FILE.
        if (log_to_file)
        {
            // If using arwain binary logging (under development) just do
            // log_file << time << data_array;
            log_file << time.time_since_epoch().count() << var << "\n";
        }

        // ADD NEW VALUES TO GLOBAL BUFFER, RESPECTING MUTEX LOCKS.
        position_buffer_lock.lock();
        position_buffer.push_back(std::array<double, 3>{var, 0, 0});
        position_buffer_lock.unlock();

        // WAIT UNTIL NEXT TIME INTERVAL.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    // IF LOGGING, CLOSE FILE HANDLES.
    if (log_to_file)
    {
        log_file.close();
    }
}

static unsigned int FALL_DETECTION_INTERVAL = 1000;

void alternate_stance_thread()
{
    using std::cout;

    Stance stance{
        CONFIG.a_threshold,
        CONFIG.crawling_threshold,
        CONFIG.running_threshold,
        CONFIG.walking_threshold,
        CONFIG.active_threshold
    };

    // cout << "Created stance\n";

    // Open file for logging
    std::ofstream freefall_file;
    if (log_to_file)
    {
        freefall_file.open(folder_date_string + "/freefall.txt");
        freefall_file << "# time freefall entanglement" << "\n";
    }

    // File handle for logging.
    std::ofstream stance_file;
    if (log_to_file)
    {
        stance_file.open(folder_date_string + "/stance.txt");
        stance_file << "# time stance" << "\n";
    }

    // cout << "Opened files\n";

    auto time = now();
    std::chrono::milliseconds interval{FALL_DETECTION_INTERVAL};

    // cout << "About to enter loop\n";

    while (!shutdown)
    {
        // cout << "Entered loop\n";
        // TODO Replicate stance and fall detection.

        // Get all relevant data.
        imu_buffer_lock.lock();
        std::deque<std::array<double, 6>> imu_data = imu_buffer;
        imu_buffer_lock.unlock();

        vel_buffer_lock.lock();
        std::deque<std::array<double, 3>> vel_data = vel_buffer;
        vel_buffer_lock.unlock();

        // cout << "Got IMU and vel buffers\n";

        stance.run(&imu_data, &vel_data);
        stance_ = stance.getStance();
        falling = stance.is_falling();
        entangled = stance.is_entangled();
        horizontal = stance.is_horizontal();

        // cout << "Did stance.run()\n";

        if (log_to_file)
        {
            freefall_file << time.time_since_epoch().count() << " " << stance.is_falling() << " " << stance.is_entangled() << "\n";
            stance_file << time.time_since_epoch().count() << " " << stance.getStance() << "\n";
        }

        // cout << "Logged to file\n";

        // Wait until the next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);

        // cout << "Passed timer, about to return to top of loop\n";
    }

}

/// mainloop
int main(int argc, char **argv)
{
    // Prepare keyboard interrupt signal handler to enable graceful exit.
    std::signal(SIGINT, sigint_handler);

    // Determine logging behaviour from command line arguments.
    arwain::InputParser input{argc, argv};
    if (input.contains("-h") || input.contains("-help"))
    {
        // Output help text.
        std::cout << "Run without arguments for no logging\n";
        std::cout << "Arguments:\n";
        std::cout << "  -lstd        Log friendly output to stdout\n";
        std::cout << "  -lfile       Record sensor data to file - files are stored in ./data_<datetime>\n";
        std::cout << "  -conf        Specify alternate configuration file\n";
        std::cout << "  -testimu     Sends IMU data (a,g) to stdout - other flags are ignored if this is set\n";
        std::cout << "  -noinf       Do not do velocity inference\n";
        std::cout << "  -h           Show this help text\n";
        std::cout << "Example usage:\n";
        std::cout << "  ./arwain -lstd -calib calib.txt -conf arwain.conf -lfile\n";
        return 1;
    }
    if (input.contains("-testimu"))
    {
        test_imu();
        return 1;
    }
    if (input.contains("-lstd"))
    {
        // Enable stdout logging.
        log_to_std = 1;
    }
    if (input.contains("-noinf"))
    {
        no_inf = 1;
    }
    if (input.contains("-lfile"))
    {
        // Enable file logging.
        std::cout << "Logging to file" << "\n";
        log_to_file = 1;
    }
    if (input.contains("-conf"))
    {
        // If alternate configuration file supplied, read it instead of default.
        config_file = input.getCmdOption("-conf");
    }
    if (!input.contains("-lstd") && !input.contains("-lfile"))
    {
        std::cerr << "No logging enabled - you probably want to use -lstd or -lfile or both" << "\n";
    }

    // Attempt to read the config file and quit if failed.
    try
    {
        CONFIG = get_configuration(config_file);
        if (log_to_std)
        {
            std::cout << "Configuration file read\n";
        }
    }
    catch (int n)
    {
        std::cout << "Problem reading configuration file\n";
        return -2;
    }
    
    // Create output directory and write copy of current configuration.
    if (log_to_file)
    {
        folder_date_string = "./data_" + datetimestring();
        if (!std::experimental::filesystem::is_directory(folder_date_string))
        {
            std::experimental::filesystem::create_directory(folder_date_string);
        }
        std::experimental::filesystem::copy(config_file, folder_date_string + "/config.conf");
    }

    // Preload buffers.
    for (unsigned int i = 0; i < IMU_BUFFER_LEN; i++)
    {
        imu_buffer.push_back(std::array<double, 6>{0, 0, 0, 0, 0, 0});
        imu_world_buffer.push_back(std::array<double, 6>{0, 0, 0, 0, 0, 0});
    }
    for (unsigned int i = 0; i < MAG_BUFFER_LEN; i++)
    {
        mag_buffer.push_back(std::array<double, 3>{0, 0, 0});
        mag_world_buffer.push_back(std::array<double, 3>{0, 0, 0});
    }
    for (unsigned int i = 0; i < VELOCITY_BUFFER_LEN; i++)
    {
        vel_buffer.push_back(std::array<double, 3>{0, 0, 0});
    }
    for (unsigned int i = 0; i < POSITION_BUFFER_LEN; i++)
    {
        position_buffer.push_back(std::array<double, 3>{0, 0, 0});
    }
    for (unsigned int i = 0; i < ORIENTATION_BUFFER_LEN; i++)
    {
        euler_orientation_buffer.push_back(euler_orientation_t{0, 0, 0});
        quat_orientation_buffer.push_back(quaternion{0, 0, 0, 0});
    }

    // Get IMU configuration from config.
    gyro_bias = {CONFIG.gyro_bias_x, CONFIG.gyro_bias_y, CONFIG.gyro_bias_z};
    accel_bias = {CONFIG.accel_bias_x, CONFIG.accel_bias_y, CONFIG.accel_bias_z};
    mag_bias = {CONFIG.mag_bias_x, CONFIG.mag_bias_y, CONFIG.mag_bias_z};
    mag_scale = {CONFIG.mag_scale_x, CONFIG.mag_scale_y, CONFIG.mag_scale_z};

    // Initialize the IMU.
    if (init_bmi270(CONFIG.use_magnetometer || CONFIG.log_magnetometer, "none") != 0)
    {
        std::cout << "IMU failed to start" << "\n";
        return -1;
    }

    // Start threads.
    std::thread imu_thread(imu_reader);
    std::thread velocity_prediction(predict_velocity);
    // std::thread fall_detection(run_fall_detect);
    // std::thread stance_detection(run_stance_detect);
    std::thread stance_thread(alternate_stance_thread);
    std::thread radio_thread(transmit_lora);
    std::thread logging_thread(std_output);
    // std::thread alert_thread(alert_system);

    // Wait for all threads to terminate.
    imu_thread.join();
    velocity_prediction.join();
    stance_thread.join();
    // fall_detection.join();
    // stance_detection.join();
    radio_thread.join();
    logging_thread.join();
    // alert_thread.join();

    return 1;
}
