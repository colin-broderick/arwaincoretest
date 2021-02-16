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

#include "quaternions.h"
#include "stance.h"
#include "imu_utils.h"
#include "madgwick.h"
#include "efaroe.h"
#include "math_util.h"
#include "rknn_api.h"
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
unsigned int VELOCITY_PREDICTION_INTERVAL = 50;
unsigned int LORA_TRANSMISSION_INTERVAL = 1000;
unsigned int ALERT_SYSTEM_INTERVAL = 1000;

// Buffer sizes.
unsigned int POSITION_BUFFER_LEN = 200;
unsigned int VELOCITY_BUFFER_LEN = 200;
unsigned int ORIENTATION_BUFFER_LEN = 200;
unsigned int IMU_BUFFER_LEN = 200;

// Default config file location.
std::string config_file = "./arwain.conf";

// Flags for whether to produce various log outputs.
int log_to_std = 0;
int log_to_file = 0;

// Name for data folder
std::string folder_date_string;

// IMU data buffers.
std::deque<std::array<float, 6>> imu_full_buffer;
std::deque<std::array<float, 6>> world_imu_buffer;
std::deque<std::array<float, 3>> vel_full_buffer;
std::deque<std::array<float, 3>> position_buffer;
std::deque<euler_orientation_t> euler_orientation_buffer;
std::deque<quaternion> quat_orientation_buffer;

extern std::string stance;
extern int horizontal;
extern int entangled;
extern int falling;
int status_flags = 0;

// Kill flag for all threads.
int shutdown = 0;

// Mutex locks.
std::mutex imu_buffer_lock;
std::mutex world_imu_buffer_lock;
std::mutex vel_buffer_lock;
std::mutex status_flag_lock;
std::mutex position_buffer_lock;
std::mutex orientation_buffer_lock;
extern std::mutex stance_lock;


void std_output()
{
    if (log_to_std)
    {
        // Set up timing, including pause while IMU warms up.
        std::chrono::milliseconds interval(1000);
        std::this_thread::sleep_for(interval*3);
        auto time = std::chrono::system_clock::now();

        // Output string built here.
        std::stringstream ss;

        while (!shutdown)
        {
            // Add position to the string stream.        
            position_buffer_lock.lock();
            ss << "Position:        (" << position_buffer.back()[0] << ", " << position_buffer.back()[1] << ", " << position_buffer.back()[2] << ")" << "\n";
            position_buffer_lock.unlock();

            // Add stance to the string stream.
            stance_lock.lock();
            
            stance_lock.unlock();

            // Add Euler and quaternion orientations to the string stream.
            orientation_buffer_lock.lock();
            ss << "Orientation (ME):(" << euler_orientation_buffer.back().roll << ", " << euler_orientation_buffer.back().pitch << ", " << euler_orientation_buffer.back().yaw << ")" << "\n";;
            ss << "Orientation (MQ):(" << quat_orientation_buffer.back().w << ", " << quat_orientation_buffer.back().x << ", " << quat_orientation_buffer.back().y << ", " << quat_orientation_buffer.back().z << ")" << "\n";
            orientation_buffer_lock.unlock();

            // Add stance to the string stream.
            std::string fall = is_falling() ? "ON" : "OFF";
            std::string entd = is_entangled() ? "ON" : "OFF";
            std::string ori = is_horizontal() ? "horizontal" : "vertical";
            ss << "Stance:          " << stance << ", " <<  ori << "\n";
            ss << "Fall flag:       " << fall << "\n";
            ss << "Entangled flag:  " << entd << "\n";

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

    // Initialize orientation filter.
    Madgwick orientation_filter{(float)(1000.0/IMU_READING_INTERVAL)};

    // eFaroe orientation_filter{
    //     quaternion{1,0,0,0},
    //     {0,0,0},
    //     0,
    //     0
    // };

    // Local buffers for IMU data
    vec_scaled_output accel_data;
    vec_scaled_output gyro_data;
    vec_scaled_output world_accel_data;
    vec_scaled_output world_gyro_data;
    euler_orientation_t euler_data;
    quaternion quat_data;

    // File handles for logging.
    std::ofstream acce_file;
    std::ofstream gyro_file;
    std::ofstream euler_file;
    std::ofstream quat_file;
    std::ofstream mag_file;

    if (log_to_file)
    {
        // Open file handles for data logging.
        acce_file.open(folder_date_string + "/acce.txt");
        gyro_file.open(folder_date_string + "/gyro.txt");
        euler_file.open(folder_date_string + "/euler_orientation.txt");
        quat_file.open(folder_date_string + "/quat_orientation.txt");
        mag_file.open(folder_date_string + "/mag.txt");

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

        // Log IMU to file.
        if (log_to_file)
        {
            acce_file << time.time_since_epoch().count() << " " << accel_data.x << " " << accel_data.y << " " << accel_data.z << "\n";
            gyro_file << time.time_since_epoch().count() << " " << gyro_data.x << " " << gyro_data.y << " " << gyro_data.z << "\n";
        }

        // Perform an orientation filter step
        orientation_filter.updateIMU(
            time.time_since_epoch().count(),
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

        // Add world-aligned IMU to its own buffer.
        world_accel_data = world_align(accel_data, quat_data);
        world_gyro_data = world_align(gyro_data, quat_data);
        world_imu_buffer_lock.lock();
        world_imu_buffer.push_back(std::array<float, 6>{
            world_accel_data.x, world_accel_data.y, world_accel_data.z,
            world_gyro_data.x, world_gyro_data.y, world_gyro_data.z
        });
        world_imu_buffer.pop_front();
        world_imu_buffer_lock.unlock();

        // Log orientation information to file.
        if (log_to_file)
        {
            euler_file << time.time_since_epoch().count() << " " << euler_data.roll << " " << euler_data.pitch << " " << euler_data.yaw << "\n";
            quat_file << time.time_since_epoch().count() << " " << quat_data.w << " " << quat_data.x << " " << quat_data.y << " " << quat_data.z << "\n";
        }
        
        // Wait until the next tick.
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

/// Periodically sends the most recent position and status information via LoRa.
int transmit_lora()
{
    // TODO: Set up radio.

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(LORA_TRANSMISSION_INTERVAL);

    std::ofstream lora_file;

    std::array<float, 3> position;
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

std::array<float, 3> integrate(std::deque<std::array<float, 6>> data, float dt, unsigned int offset = 0)
{
    std::array<float, 3> acc_mean = {-0.182194123, -0.59032666517, 9.86202363151991};
    std::array<float, 3> integrated_data = {0, 0, 0};
    for (unsigned int i = 0; i < data.size(); i++)
    {
        for (unsigned int j = 0; j < 3; j++)
        {
            integrated_data[j] = integrated_data[j] + (data[i][j + offset] - acc_mean[j]) * dt;
        }
    }
    return integrated_data;
}

/// Sets up and runs velocity prediction using the NPU. Run as a thread.
int predict_velocity()
{
    // TODO: Set up NPU and feed in model.
    // TODO: Merge the inference code into this function. Will need further abstraction.
    std::chrono::milliseconds presleep(1000);
    std::this_thread::sleep_for(presleep*3);

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(VELOCITY_PREDICTION_INTERVAL);

    // TEST Get the filter weight parameter(s) from the config file.
    float npu_weight = arwain::get_config<float>(config_file, "npu_vel_weight_confidence");

    // Initialize buffers to contain working values.
    std::array<float, 3> vel;                    // The sum of npu_vel and imu_vel_delta.
    std::array<float, 3> npu_vel;                // To hold the neural network prediction of velocity.
    std::array<float, 3> vel_previous;           // Contains the velocity calculation from the previous loop.
    std::array<float, 3> npu_vel_delta;          // Stores the difference between the npu velocity prediction and vel_previous.
    std::array<float, 3> imu_vel_delta;          // To integrate the velocity based on IMU readings.
    std::array<float, 3> pos;
    std::deque<std::array<float, 6>> imu;        // To contain the last second of IMU data.
    std::deque<std::array<float, 6>> imu_latest; // To contain the last VELOCITY_PREDICTION_INTERVAL of IMU data.

    // File handles for logging.
    std::ofstream position_file;
    std::ofstream velocity_file;

    // Time in seconds between inferences.
    float interval_seconds = ((float)(VELOCITY_PREDICTION_INTERVAL))/1000.0;

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

    while (!shutdown)
    {
        // Grab latest IMU packet
        imu_buffer_lock.lock();
        imu = imu_full_buffer;
        imu_buffer_lock.unlock();

        // TODO: Make velocity prediction
        npu_vel = std::array<float, 3>{0, 0, 0};
        
        // TEST Find the change in velocity from the last period, as predicted by the npu.
        npu_vel_delta = npu_vel - vel_previous;

        // TEST Get last interval worth of IMU data.
        imu_latest = {imu.end() - backtrack, imu.end()};

        // TEST Single integrate the small IMU slice to get delta-v over the period.
        imu_vel_delta = integrate(imu_latest, interval_seconds );

        // TEST Weighted combination of velocity deltas from NPU and IMU integration.
        vel = npu_vel_delta*npu_weight + imu_vel_delta*(1-npu_weight);

        // TEST Add the filtered delta onto the previous vel estimate and add to buffer.
        vel = vel + vel_previous;
        vel_buffer_lock.lock();
        vel_full_buffer.push_back(vel);
        vel_full_buffer.pop_front();
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

    return 1;
}

/// Captures keyboard interrupt to set shutdown flag.
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

/// This function is never called. It should be considered a template
/// for implementing new threaded functionality. All existing threads
/// follow approximately this design pattern.
void thread_template()
{
    // DECLARE ANY LOCAL VARIABLES OUTSIDE THE WHILE LOOP.
    float var;
    float from_global;

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
        position_buffer.push_back(std::array<float, 3>{var, 0, 0});
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

/// mainloop
int main(int argc, char **argv)
{
    // Prepare keyboard interrupt signal handler to enable graceful exit.
    std::signal(SIGINT, sigint_handler);

    // Determine logging behaviour from command line arguments.
    arwain::InputParser input(argc, argv);
    if (input.contains("-h") || input.contains("-help"))
    {
        // Output help text.
        std::cout << "Run without arguments for no logging\n";
        std::cout << "Arguments:\n";
        std::cout << "  -lstd        Log friendly output to stdout\n";
        std::cout << "  -lfile       Record sensor data to file - files are stored in ./data_<datetime>\n";
        std::cout << "  -conf        Specify alternate configuration file\n";
        std::cout << "  -testimu     Sends IMU data (a,g) to stdout - other flags are ignored if this is set\n";
        std::cout << "  -h           Show this help text\n";
        std::cout << "\n";

        exit(1);
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

    // Create output directory if it doesn't already exist.
    if (log_to_file)
    {
        folder_date_string = "./data_" + datetimestring();
        if (!std::experimental::filesystem::is_directory(folder_date_string))
        {
            std::experimental::filesystem::create_directory(folder_date_string);
        }
    }

    // Preload buffers.
    for (unsigned int i = 0; i < IMU_BUFFER_LEN; i++)
    {
        imu_full_buffer.push_back(std::array<float, 6>{0, 0, 0, 0, 0, 0});
        world_imu_buffer.push_back(std::array<float, 6>{0, 0, 0, 0, 0, 0});
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
        quat_orientation_buffer.push_back(quaternion{0, 0, 0, 0});
    }

    // Start threads
    std::thread imu_full_thread(bmi270_reader);
    std::thread velocity_prediction(predict_velocity);
    std::thread fall_detection(run_fall_detect);
    std::thread stance_detection(run_stance_detect);
    std::thread radio_thread(transmit_lora);
    std::thread alert_thread(alert_system);
    std::thread logging_thread(std_output);

    // Wait for all threads to terminate.
    imu_full_thread.join();
    velocity_prediction.join();
    fall_detection.join();
    stance_detection.join();
    radio_thread.join();
    alert_thread.join();
    logging_thread.join();

    return 1;
}
