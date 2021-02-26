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

extern configuration CONFIG;
extern int shutdown;
extern std::string folder_date_string;
extern std::mutex imu_buffer_lock;
extern std::mutex vel_buffer_lock;
extern std::deque<std::array<double, 6>> imu_buffer;
extern std::deque<std::array<double, 3>> vel_buffer;

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
std::string stance_;

// Configuration file location.
extern std::string config_file;

std::mutex fall_lock;
std::mutex stance_lock;

std::thread fall_thread;
std::thread stance_thread;

int Stance::is_falling()
{
    fall_lock.lock();
    int ret = m_falling;
    m_falling = 0;
    fall_lock.unlock();
    return ret;
}

// Check if entangled flag set.
int Stance::is_entangled()
{
    fall_lock.lock();
    int ret = m_entangled;
    m_entangled = 0;
    fall_lock.unlock();
    return ret;
}

// Calculates the mean magnitude of a (x,3) sized deque for doubles.
double Stance::buffer_mean_magnitude(std::deque<std::array<double, 3>> buffer)
{
    double mean = 0.0;
    for (unsigned int i=0; i < buffer.size(); i++)
    {
        double square_sum = 0;
        for (unsigned int j=0; j < 3; j++)
        {
            square_sum += buffer[i][j]*buffer[i][j];
        }
        mean += sqrt(square_sum);
    }
    mean /= buffer.size();
    return mean;
}

// Calculates the mean magnitude of a (x,3) sized vector for doubles.
double Stance::buffer_mean_magnitude(std::vector<std::array<double, 3>> buffer)
{
    double mean = 0.0;
    for (unsigned int i=0; i < buffer.size(); i++)
    {
        double square_sum = 0;
        for (unsigned int j=0; j < 3; j++)
        {
            square_sum += buffer[i][j]*buffer[i][j];
        }
        mean += sqrt(square_sum);
    }
    mean /= buffer.size();
    return mean;
}

/// Calculates the mean of a vector of doubles.
double Stance::vector_mean(std::vector<double> values)
{
    double mean = 0;
    for (unsigned int i = 0; i < values.size(); i++)
    {
        mean += values[i];
    }
    return mean/values.size();
}

Stance::Stance(double a_threshold, double crawling_threshold, double running_threshold, double walking_threshold, double active_threshold)
{
    m_a_threshold = a_threshold;
    m_crawling_threshold = crawling_threshold;
    m_running_threshold = running_threshold;
    m_walking_threshold = walking_threshold;
    m_active_threshold = active_threshold;
    // Make m_struggle_window have length 10.
    for (unsigned int i = 0; i < 10; i++)
    {
        m_struggle_window.push_back(0);
    }
}

void Stance::run(std::deque<std::array<double, 6>> imu_data, std::deque<std::array<double, 3>> vel_data)
{
    std::vector<std::array<double, 3>> accel_data;
    std::vector<std::array<double, 3>> gyro_data;

    for (int i = 0; i < 20; i++)
    {
        accel_data.push_back(std::array<double, 3>{
            imu_data[i][0], imu_data[i][1], imu_data[i][2]
        });
        gyro_data.push_back(std::array<double, 3>{
            imu_data[i][3], imu_data[i][4], imu_data[i][5]
        });
    }

    m_a_mean_magnitude = buffer_mean_magnitude(accel_data);
    m_g_mean_magnitude = buffer_mean_magnitude(gyro_data);
    m_v_mean_magnitude = buffer_mean_magnitude(vel_data);

    // Determine which axis has the largest value of acceleration. If not the same as vertical axis, subject must be horizontal.
    std::array<double, 3> accel_means = get_means(accel_data);
    int m_primary_axis = biggest_axis(accel_means);
    stance_lock.lock();
    if (m_primary_axis != m_vertical_axis)
    {
        m_horizontal = 1;
    }
    else
    {
        m_horizontal = 0;
    }
    stance_lock.unlock();

    // If the axis with the highest average speed is the same as the vertical axis, subject must be climbing.
    // If the speed on the vertical axis exceed the climbing threshold, the subject must be climibing.
    auto speed_means = get_means(vel_data);
    m_speed_axis = biggest_axis(speed_means);
    stance_lock.lock();
    if (m_speed_axis == m_primary_axis || speed_means[m_vertical_axis] > m_climbing_threshold)
    {
        m_climbing = 1;
    }
    else
    {
        m_climbing = 0;
    }
    stance_lock.unlock();

    if (m_a_mean_magnitude < m_a_threshold)
    {
        fall_lock.lock();
        m_falling = 1;
        fall_lock.unlock();
    }
    m_a_twitch = abs(m_a_mean_magnitude - m_gravity);
    m_tmp_struggle = (m_a_twitch + m_g_mean_magnitude) / (m_v_mean_magnitude + m_sfactor);
    m_struggle_window[m_count] = m_tmp_struggle;
    m_struggle = vector_mean(m_struggle_window);
    if (m_struggle > m_struggle_threshold)
    {
        fall_lock.lock();
        m_entangled = 1;
        fall_lock.unlock();
    }

    m_act = activity(m_a_mean_magnitude, m_g_mean_magnitude, m_v_mean_magnitude);

    // Horizontal and slow => inactive
    // Horizontal and fast => crawling
    // Vertical and slow with low activity => inactive
    // Vertical and slow with high activity => searching
    // Vertical and moderate speed => walking
    // Vertical and high speed => running
    stance_lock.lock();
    if (m_horizontal)
    {
        if (m_v_mean_magnitude < m_crawling_threshold)
        {
            m_stance = "inactive";
        }
        else if (m_v_mean_magnitude >= m_crawling_threshold)
        {
            m_stance = "crawling";
        }
    }
    else if (!m_horizontal)
    {
        if (m_v_mean_magnitude < m_walking_threshold)
        {
            if (m_act < m_active_threshold)
            {
                m_stance = "inactive";
            }
            else if (m_act >= m_active_threshold)
            {
                m_stance = "searching";
            }
        }
        else if (m_v_mean_magnitude < m_running_threshold)
        {
            m_stance = "walking";
        }
        else if (m_v_mean_magnitude >= m_running_threshold)
        {
            m_stance = "running";
        }
    }
    stance_lock.unlock();

    // Log current time and status to file.
    // if (log_to_file)
    // {
    //     freefall_file << time.time_since_epoch().count() << " " << falling << " " << entangled << "\n";
    // }

    m_count = (m_count + 1) % 10;

}

// Gives a measure of intensity of activity based on values of a, g, v.
double Stance::activity(double a, double g, double v)
{
    // TODO Need to discover a decent metric of `activity`.
    // The metric should read high when accelerations are high,
    // or gyroscope readings are high, or both acceleration and
    // gyro are high. It should read low when acceleration and gyro
    // are low.

    // This metric isn't great since you can have very high acceleration with
    // near-zero gyration, incorrectly resulting is a low activity reading.
    // Perhaps max(acceleration, gyration, acceleration*gyration)
    return a*g+v-v;
}

// Returns the index of the largest element in a 3-array of doubles. Think np.argmax().
int Stance::biggest_axis(std::array<double, 3> arr)
{
    int axis;
    if (arr[0] > arr[1] && arr[0] > arr[2])
    {
        axis = 0;
    }
    else if (arr[1] > arr[0] && arr[1] > arr[2])
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
int Stance::is_horizontal()
{  
    stance_lock.lock();
    int ret = m_horizontal;
    stance_lock.unlock();
    return ret;
}

std::array<double, 3> Stance::get_means(std::deque<std::array<double, 3>> source_vector)
{
    std::array<double, 3> ret;
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

std::array<double, 6> Stance::get_means(std::deque<std::array<double, 6>> source_vector)
{
    std::array<double, 6> ret;
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
std::array<double, 3> Stance::get_means(std::vector<std::array<double, 3>> source_vector)
{
    std::array<double, 3> ret;
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

// Returns integer representing current stance.
std::string Stance::getStance()
{
    stance_lock.lock();
    std::string ret = m_stance;
    stance_lock.unlock();
    return ret;
}
