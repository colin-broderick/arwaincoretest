#include <iostream>
#include <mutex>
#include <chrono>
#include <vector>
#include <thread>
#include <math.h>
#include <array>
#include <deque>
#include <fstream>

#include "stance.h"
#include "utils.h"

extern int shutdown;
extern std::string folder_date_string;
extern std::mutex imu_buffer_lock;
extern std::mutex vel_buffer_lock;
extern std::deque<std::array<float, 6>> imu_full_buffer;
extern std::deque<std::array<float, 3>> vel_full_buffer;

// Flags for logging behaviour, declared and defined in main.cpp.
extern int log_to_file;
extern int log_to_std;

// Time intervals, all in milliseconds.
unsigned int STANCE_DETECTION_INTERVAL = 1000;
unsigned int FALL_DETECTION_INTERVAL = 100;

// Status flags.
int die = 0;
int falling = 0;
int entangled = 0;
int horizontal = 0;
int climbing = 0;
std::string stance;

// Configuration file location.
extern std::string config_file;

std::mutex fall_lock;
std::mutex stance_lock;

std::thread fall_thread;
std::thread stance_thread;

// Get current time.
std::chrono::_V2::system_clock::time_point now()
{
    return std::chrono::system_clock::now();
}

// Check if falling flag set.
int is_falling()
{
    fall_lock.lock();
    int ret = falling;
    falling = 0;
    fall_lock.unlock();
    return ret;
}

// Check if entangled flag set.
int is_entangled()
{
    fall_lock.lock();
    int ret = entangled;
    entangled = 0;
    fall_lock.unlock();
    return ret;
}

// Calculates the mean magnitude of a (x,3) sized deque for floats.
float buffer_mean_magnitude(std::deque<std::array<float, 3>> buffer)
{
    float mean = 0.0;
    for (unsigned int i=0; i < buffer.size(); i++)
    {
        float square_sum = 0;
        for (unsigned int j=0; j < 3; j++)
        {
            square_sum += buffer[i][j]*buffer[i][j];
        }
        mean += sqrt(square_sum);
    }
    mean /= buffer.size();
    return mean;
}

// Calculates the mean magnitude of a (x,3) sized vector for floats.
float buffer_mean_magnitude(std::vector<std::array<float, 3>> buffer)
{
    float mean = 0.0;
    for (unsigned int i=0; i < buffer.size(); i++)
    {
        float square_sum = 0;
        for (unsigned int j=0; j < 3; j++)
        {
            square_sum += buffer[i][j]*buffer[i][j];
        }
        mean += sqrt(square_sum);
    }
    mean /= buffer.size();
    return mean;
}

/// Calculates the mean of a vector of floats.
float vector_mean(std::vector<float> values)
{
    float mean = 0;
    for (unsigned int i = 0; i < values.size(); i++)
    {
        mean += values[i];
    }
    return mean/values.size();
}

// This is the fall detection main loop. Periodically check 'die' to decide whether to quit.
int run_fall_detect()
{
    float a_mean_magnitude;
    float g_mean_magnitude;
    float v_mean_magnitude;

    // Read thresholds and constants from config file.
    float a_threshold = get_config<float>(config_file, "a_threshold");
    float struggle_threshold = get_config<float>(config_file, "struggle_threshold");
    float gravity = get_config<float>(config_file, "gravity");

    float a_twitch;
    float tmp_struggle;

    float sfactor = 1;

    int count = 0;
    float struggle = 0;
    
    // Open file for logging
    std::ofstream freefall_file;
    if (log_to_file)
    {
        freefall_file.open(folder_date_string + "/freefall.txt");
        freefall_file << "# time freefall entanglement" << "\n";
    }

    std::vector<float> struggle_window(10);

    auto time = now();
    std::chrono::milliseconds interval(FALL_DETECTION_INTERVAL);

    while (!shutdown)
    {
        // Grab the imu_data to be used, i.e. most recent second of imu_data.
        imu_buffer_lock.lock();
        std::deque<std::array<float, 6>> imu_data = imu_full_buffer;
        imu_buffer_lock.unlock();

        vel_buffer_lock.lock();
        std::deque<std::array<float, 3>> vel_data = vel_full_buffer;
        vel_buffer_lock.unlock();

        // Separate accel and gyro buffers; bit more convenient this way.
        std::vector<std::array<float, 3>> accel_data;
        std::vector<std::array<float, 3>> gyro_data;
        for (int i = 0; i < 20; i++)
        {
            accel_data.push_back(std::array<float, 3>{
                imu_data[i][0], imu_data[i][1], imu_data[i][2]
            });
            gyro_data.push_back(std::array<float, 3>{
                imu_data[i][3], imu_data[i][4], imu_data[i][5]
            });
        }

        a_mean_magnitude = buffer_mean_magnitude(accel_data);
        g_mean_magnitude = buffer_mean_magnitude(gyro_data);
        v_mean_magnitude = buffer_mean_magnitude(vel_data);

        if (a_mean_magnitude < a_threshold)
        {
            fall_lock.lock();
            falling = 1;
            fall_lock.unlock();
        }
        a_twitch = abs(a_mean_magnitude - gravity);
        tmp_struggle = (a_twitch + g_mean_magnitude) / (v_mean_magnitude + sfactor);
        struggle_window[count] = tmp_struggle;
        struggle = vector_mean(struggle_window);
        if (struggle > struggle_threshold)
        {
            fall_lock.lock();
            entangled = 1;
            fall_lock.unlock();
        }

        // Log current time and status to file.
        if (log_to_file)
        {
            freefall_file << time.time_since_epoch().count() << " " << falling << " " << entangled << "\n";
        }

        count = (count + 1) % 10;

        // Wait until the next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    if (log_to_file)
    {
        freefall_file.close();
    }

    return 1;
}

// Gives a measure of intensity of activity based on values of a, g, v.
float activity(float a, float g, float v)
{
    // TODO Need to discover a decent metric of `activity`.
    // The metric should read high when accelerations are high,
    // or gyroscope readings are high, or both acceleration and
    // gyro are high. It should read low when acceleration and gyro
    // are low.

    // This metric isn't great since you can have very high acceleration with
    // near-zero gyration, incorrectly resulting is a low activity reading.
    // Perhaps max(acceleration, gyration, acceleration*gyration)
    return a*g;
}

// Returns the index of the largest element in a 3-array of floats. Think np.argmax().
int biggest_axis(std::array<float, 3> arr)
{
    int axis;
    if (arr[0] > arr[1] && arr[0] > arr[2])
    {
        axis = 0;
    }
    else if(arr[1] > arr[0] && arr[1] > arr[2])
    {
        axis = 1;
    }
    else
    {
        axis = 2;
    }
    return axis;
}

// Check whether subject is horizontal or vertical.
int is_horizontal()
{  
    stance_lock.lock();
    int ret = horizontal;
    stance_lock.unlock();
    return ret;
}

std::array<float, 3> get_means(std::deque<std::array<float, 3>> source_vector)
{
    std::array<float, 3> ret;
    unsigned int length = source_vector.size();
    for (unsigned int i = 0; i < length; i++)
    {
        ret[0] += abs(source_vector[i][0]);
        ret[1] += abs(source_vector[i][1]);
        ret[2] += abs(source_vector[i][2]);
    }
    ret[0] /= length;
    ret[1] /= length;
    ret[2] /= length;

    return ret;
}

std::array<float, 6> get_means(std::deque<std::array<float, 6>> source_vector)
{
    std::array<float, 6> ret;
    unsigned int length = source_vector.size();
    for (unsigned int i = 0; i < length; i++)
    {
        ret[0] += abs(source_vector[i][0]);
        ret[1] += abs(source_vector[i][1]);
        ret[2] += abs(source_vector[i][2]);
        ret[3] += abs(source_vector[i][3]);
        ret[4] += abs(source_vector[i][4]);
        ret[5] += abs(source_vector[i][5]);
    }
    ret[0] /= length;
    ret[1] /= length;
    ret[2] /= length;
    ret[3] /= length;
    ret[4] /= length;
    ret[5] /= length;

    return ret;
}

// Return the column-wise means of a size (x, 3) vector
std::array<float, 3> get_means(std::vector<std::array<float, 3>> source_vector)
{
    std::array<float, 3> ret;
    unsigned int length = source_vector.size();
    for (unsigned int i = 0; i < length; i++)
    {
        ret[0] += abs(source_vector[i][0]);
        ret[1] += abs(source_vector[i][1]);
        ret[2] += abs(source_vector[i][2]);
    }
    ret[0] /= length;
    ret[1] /= length;
    ret[2] /= length;

    return ret;
}

// This is the stance detection main loop. Periodically check value of die to decide whether to quit.
int run_stance_detect()
{
    float a_mean_magnitude;
    float g_mean_magnitude;
    float v_mean_magnitude;

    float act;

    // Read thresholds and constants from config file.
    float climbing_threshold = get_config<float>(config_file, "climbing_threshold");
    float crawling_threshold = get_config<float>(config_file, "crawling_threshold");
    float running_threshold = get_config<float>(config_file, "running_threshold");
    float walking_threshold = get_config<float>(config_file, "walking_threshold");
    float active_threshold = get_config<float>(config_file, "active_threshold");

    int vertical_axis = 1;
    int speed_axis;

    // File handle for logging.
    std::ofstream stance_file;
    if (log_to_file)
    {
        stance_file.open(folder_date_string + "/stance.txt");
        stance_file << "# time stance" << "\n";
    }
    
    auto time = now();
    std::chrono::milliseconds interval(STANCE_DETECTION_INTERVAL);

    while (!shutdown)
    {
        // Grab the imu_data to be used, i.e. most recent second of imu_data.
        imu_buffer_lock.lock();
        std::deque<std::array<float, 6>> imu_data = imu_full_buffer;
        imu_buffer_lock.unlock();

        vel_buffer_lock.lock();
        std::deque<std::array<float, 3>> vel_data = vel_full_buffer;
        vel_buffer_lock.unlock();

        // Separate accel and gyro buffers; bit more convenient this way.
        std::vector<std::array<float, 3>> accel_data;
        std::vector<std::array<float, 3>> gyro_data;
        for (unsigned int i = 0; i < imu_data.size(); i++)
        {
            accel_data.push_back(std::array<float, 3>{
                imu_data[i][0], imu_data[i][1], imu_data[i][2]
            });
            gyro_data.push_back(std::array<float, 3>{
                imu_data[i][3], imu_data[i][4], imu_data[i][5]
            });
        }

        // Determine which axis has the largest value of acceleration. If not the same as vertical axis, subject must be horizontal.
        std::array<float, 3> accel_means = get_means(accel_data);
        int primary_axis = biggest_axis(accel_means);
        stance_lock.lock();
        if (primary_axis != vertical_axis)
        {
            horizontal = 1;
        }
        else
        {
            horizontal = 0;
        }
        stance_lock.unlock();

        // If the axis with the highest average speed is the same as the vertical axis, subject must be climbing.
        // If the speed on the vertical axis exceed the climbing threshold, the subject must be climibing.
        auto speed_means = get_means(vel_data);
        speed_axis = biggest_axis(speed_means);
        stance_lock.lock();
        if (speed_axis == primary_axis || speed_means[vertical_axis] > climbing_threshold)
        {
            climbing = 1;
        }
        else
        {
            climbing = 0;
        }
        stance_lock.unlock();

        a_mean_magnitude = buffer_mean_magnitude(accel_data);
        g_mean_magnitude = buffer_mean_magnitude(gyro_data);
        v_mean_magnitude = buffer_mean_magnitude(vel_data);
        act = activity(a_mean_magnitude, g_mean_magnitude, v_mean_magnitude);

        // Horizontal and slow => inactive
        // Horizontal and fast => crawling
        // Vertical and slow with low activity => inactive
        // Vertical and slow with high activity => searching
        // Vertical and moderate speed => walking
        // Vertical and high speed => running
        stance_lock.lock();
        if (horizontal)
        {
            if (v_mean_magnitude < crawling_threshold)
            {
                stance = "inactive";
            }
            else if (v_mean_magnitude >= crawling_threshold)
            {
                stance = "crawling";
            }
        }
        else if (!horizontal)
        {
            if (v_mean_magnitude < walking_threshold)
            {
                if (act < active_threshold)
                {
                    stance = "inactive";
                }
                else if (act >= active_threshold)
                {
                    stance = "searching";
                }
            }
            else if (v_mean_magnitude < running_threshold)
            {
                stance = "walking";
            }
            else if (v_mean_magnitude >= running_threshold)
            {
                stance = "running";
            }
        }
        stance_lock.unlock();

        if (log_to_file)
        {
            stance_file << time.time_since_epoch().count() << " " << stance << "\n";
        }

        // Wait until the next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    return 1;
}

// Returns integer representing current stance.
int get_stance()
{
    int ret;
    stance_lock.lock();
    if (stance == "inactive")
    {
        ret = 0;
    }
    else if (stance == "walking")
    {
        ret = 1;
    }
    else if (stance == "searching")
    {
        ret = 2;
    }
    else if (stance == "crawling")
    {
        ret = 3;
    }
    else if (stance == "running")
    {
        ret = 4;
    }
    else
    {
        ret = 5;
    }
    stance_lock.unlock();
    return ret;
}
