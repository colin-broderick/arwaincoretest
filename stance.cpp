#include <iostream>
#include <mutex>
#include <chrono>
#include <vector>
#include <math.h>
#include <array>
#include <deque>


#include "Kernel.h"
#include "mbed.h"
#include "stance.h"

extern int shutdown;
extern Mutex full_buffer_lock;
extern std::deque<std::array<float, 6>> imu_full_buffer;

// Buffers for storing acceleration, gyroscope, and velocity data.
std::vector<std::array<float, 3>> a_fall_buffer;
std::vector<std::array<float, 3>> g_fall_buffer;
std::vector<std::array<float, 3>> v_fall_buffer;

std::vector<std::array<float, 3>> a_stance_buffer;
std::vector<std::array<float, 3>> g_stance_buffer;
std::vector<std::array<float, 3>> v_stance_buffer;

// State flags.
int die = 0;
int falling = 0;
int entangled = 0;
int horizontal = 0;
int climbing = 0;
std::string stance;

// Thresholds and constants.
std::chrono::milliseconds freefall_dt{100};  // Delay between freefall/entanglement assessments.
std::chrono::milliseconds stance_dt{1000};   // Delay between stance assessments.
float a_threshold = 3;
float struggle_threshold = 3;
float gravity = 9.8;
float climbing_threshold = 1;
float crawling_threshold = 0.5;
float running_threshold = 3;
float walking_threshold = 0.5;
float active_threshold = 20;

Mutex fall_lock;
Mutex stance_lock;
Kernel::Clock::time_point last_update;

Thread fall_thread;
Thread stance_thread;


// Get current time.
Kernel::Clock::time_point now()
{
    return Kernel::Clock::now();
}

// Update the internal data buffer with a new entry.
int update(float ax, float ay, float az, float gx, float gy, float gz, float vx, float vy, float vz)
{
    last_update = now();
    fall_lock.lock();
    a_fall_buffer.push_back(std::array<float, 3>{ax, ay, az});
    g_fall_buffer.push_back(std::array<float, 3>{gx, gy, gz});
    v_fall_buffer.push_back(std::array<float, 3>{vx, vy, vz});
    fall_lock.unlock();

    stance_lock.lock();
    a_stance_buffer.push_back(std::array<float, 3>{ax, ay, az});
    g_stance_buffer.push_back(std::array<float, 3>{gx, gy, gz});
    v_stance_buffer.push_back(std::array<float, 3>{vx, vy, vz});
    stance_lock.unlock();

    return 1;
}

// Clear the buffers use for detecting stance.
int empty_stance_buffers()
{
    a_stance_buffer.clear();
    g_stance_buffer.clear();
    v_stance_buffer.clear();
    return 1;
}

// Clear the buffers used for detecting freefall and entanglement.
int empty_fall_buffers()
{
    a_fall_buffer.clear();
    g_fall_buffer.clear();
    v_fall_buffer.clear();
    return 1;
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

// Empty all buffers and reset all flags.
int reset()
{
    fall_lock.lock();
    stance_lock.lock();

    a_fall_buffer.clear();
    g_fall_buffer.clear();
    v_fall_buffer.clear();
    a_stance_buffer.clear();
    g_stance_buffer.clear();
    v_stance_buffer.clear();
    falling = 0;
    entangled = 0;
    horizontal = 0;
    climbing = 0;
    stance = "inactive";

    fall_lock.unlock();
    stance_lock.unlock();
    return 1;
}

// Calculates the mean magnitude of a (x,3) sized vector for floats.
float buffer_mean(std::vector<std::array<float, 3>> buffer)
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

// Calculates the mean of a vector of floats.
float struggle_mean(std::vector<float> struggles)
{
    float mean = 0;
    for (unsigned int i = 0; i < struggles.size(); i++)
    {
        mean += struggles[i];
    }
    return mean/struggles.size();
}

// This is the fall detection main loop. Periodically check 'die' to decide whether to quit.
int run_fall_detect()
{
    float a_mean_magnitude;
    float g_mean_magnitude;
    float v_mean_magnitude;

    float a_twitch;
    float tmp_struggle;

    float sfactor = 1;

    int count = 0;
    float struggle = 0;

    std::vector<float> struggle_window(10);

    while (!die)
    {
        ThisThread::sleep_for(freefall_dt);

        fall_lock.lock();
        a_mean_magnitude = buffer_mean(a_fall_buffer);
        g_mean_magnitude = buffer_mean(g_fall_buffer);
        v_mean_magnitude = buffer_mean(v_fall_buffer);

        if (a_mean_magnitude < a_threshold)
        {
            falling = 1;
        }
        a_twitch = abs(a_mean_magnitude - gravity);
        tmp_struggle = (a_twitch + g_mean_magnitude) / (v_mean_magnitude + sfactor);
        struggle_window[count] = tmp_struggle;
        struggle = struggle_mean(struggle_window);
        if (struggle > struggle_threshold)
        {
            entangled = 1;
        }

        count = (count + 1) % 10;

        empty_fall_buffers();

        fall_lock.unlock();
    }

    return 1;
}

// Gives a measure of intensity of activity based on values of a, g, v.
float activity(float a, float g, float v)
{
    // TODO
    return a*g;
}

// Returns the index of the largest element in a 3-array of floats. Think np.argmax().
int biggest_axis(float arr[3])
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
    return horizontal;
}

// Return the column-wise means of a size (x, 3) vector
int get_means(float arr[], std::vector<std::array<float, 3>> source_vector)
{
    unsigned int length = source_vector.size();
    for (unsigned int i = 0; i < length; i++)
    {
        arr[0] += abs(v_stance_buffer[i][0]);
        arr[1] += abs(v_stance_buffer[i][1]);
        arr[2] += abs(v_stance_buffer[i][2]);
    }
    arr[0] /= length;
    arr[1] /= length;
    arr[2] /= length;

    return 1;
}

// This is the stance detection main loop. Periodically check value of die to decide whether to quit.
int run_stance_detect()
{
    float a_mean_magnitude;
    float g_mean_magnitude;
    float v_mean_magnitude;

    float act;

    int vertical_axis = 2;
    int speed_axis;

    auto time = Kernel::Clock::now();
    auto interval = 1000 * 1ms;

    while (!shutdown)
    {
        // Grab the data to be used, i.e. most recent second of data.
        full_buffer_lock.lock();
        std::deque<std::array<float, 6>> data = imu_full_buffer;

        // Determine which axis has the largest value of acceleration. If not the same as vertical axis, subject must be horizontal.
        float accel_means[3];
        get_means(accel_means, gyro_meansimu_full_buffer);
        int primary_axis = biggest_axis(accel_means);
        if (primary_axis != vertical_axis)
        {
            horizontal = 1;
        }
        else
        {
            horizontal = 0;
        }

        // If the axis with the highest average speed is the same as the vertical axis, subject must be climbing.
        // If the speed on the vertical axis exceed the climbing threshold, the subject must be climibing.
        float speed_means[3];
        get_means(speed_means, v_stance_buffer);
        speed_axis = biggest_axis(speed_means);
        if (speed_axis == primary_axis || speed_means[vertical_axis] > climbing_threshold)
        {
            climbing = 1;
        }
        else
        {
            climbing = 0;
        }

        a_mean_magnitude = buffer_mean(a_stance_buffer);
        g_mean_magnitude = buffer_mean(g_stance_buffer);
        v_mean_magnitude = buffer_mean(v_stance_buffer);
        act = activity(a_mean_magnitude, g_mean_magnitude, v_mean_magnitude);

        // Horizontal and slow => inactive
        // Horizontal and fast => crawling
        // Vertical and slow with low activity => inactive
        // Vertical and slow with high activity => searching
        // Vertical and moderate speed => walking
        // Vertical and high speed => running
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

        // Reset and release buffers.
        empty_stance_buffers();
        stance_lock.unlock();

        // Wait until the next tick.
        time = time + interval;
        ThisThread::sleep_until(time);
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

// Set the value of 'die' to 1 to instruct the detector to stop.
int kill()
{
    die = 1;
    return 1;
}

// Create detector threads and start spinning.
int start()
{
    last_update = now();
    Thread fall_thread;
    Thread stance_thread;
    fall_thread.start(run_fall_detect);
    stance_thread.start(run_stance_detect);

    printf("Detector threads started\n");

    while (!die)
    {
        ThisThread::sleep_for(1s);
    }
    
    fall_thread.join();
    stance_thread.join();

    printf("Detector threads ended\n");

    return 1;
}
