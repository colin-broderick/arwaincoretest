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

// Constructors -----------------------------------------------------------------------------------

/** \brief Constructor for the stance class.
 * \param fall_threshold Acceleration magnitude below which a fall event is detected.
 * \param crawling_threshold Speed below which, if horizontal, stance is detected as crawling.
 * \param running_threshold Speed above which, if vertical, stance is detected as running.
 * \param walking_threshold Speed above which, if vertical, stance is detected as walking.
 * \param active_threshold Value of the internal activity metric above which an entanglement event is detected.
 */
StanceDetector::StanceDetector(double fall_threshold, double crawling_threshold, double running_threshold, double walking_threshold, double active_threshold, double struggle_threshold)
{
    m_fall_threshold = fall_threshold;
    m_crawling_threshold = crawling_threshold;
    m_running_threshold = running_threshold;
    m_walking_threshold = walking_threshold;
    m_active_threshold = active_threshold;
    m_struggle_threshold = struggle_threshold;
    
    // Make m_struggle_window have length 10.
    for (unsigned int i = 0; i < 10; i++)
    {
        m_struggle_window.push_back(0);
    }
}

// General methods --------------------------------------------------------------------------------

/** \brief Run detection algorithms against provided sensor data.
 * \param imu_data Pointer to deq<arr<double>> containging acceleration and gyro data.
 * \param vel_data Pointer to deq<arr<double>> containing velocity data.
 */
void StanceDetector::run(const std::deque<std::array<double, 6>> &imu_data, const std::deque<std::array<double, 3>> &vel_data)
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
        m_attitude = Horizontal;
    }
    else
    {
        m_attitude = Vertical;
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

    if (m_a_mean_magnitude < m_fall_threshold)
    {
        fall_lock.lock();
        m_falling = Falling;
        fall_lock.unlock();
    }
    m_a_twitch = abs(m_a_mean_magnitude - m_gravity);
    m_tmp_struggle = (m_a_twitch + m_g_mean_magnitude) / (m_v_mean_magnitude + m_sfactor);
    m_struggle_window[m_count] = m_tmp_struggle;
    m_struggle = vector_mean(m_struggle_window);
    if (m_struggle > m_struggle_threshold)
    {
        fall_lock.lock();
        m_entangled = Entangled;
        fall_lock.unlock();
    }

    m_activity = activity(m_a_mean_magnitude, m_g_mean_magnitude, m_v_mean_magnitude);

    // Horizontal and slow => inactive
    // Horizontal and fast => crawling
    // Vertical and slow with low activity => inactive
    // Vertical and slow with high activity => searching
    // Vertical and moderate speed => walking
    // Vertical and high speed => running
    stance_lock.lock();
    if (m_attitude)
    {
        if (m_v_mean_magnitude < m_crawling_threshold)
        {
            m_stance = Inactive;
        }
        else if (m_v_mean_magnitude >= m_crawling_threshold)
        {
            m_stance = Crawling;
        }
    }
    else if (!m_attitude)
    {
        if (m_v_mean_magnitude < m_walking_threshold)
        {
            if (m_activity < m_active_threshold)
            {
                m_stance = Inactive;
            }
            else if (m_activity >= m_active_threshold)
            {
                m_stance = Searching;
            }
        }
        else if (m_v_mean_magnitude < m_running_threshold)
        {
            m_stance = Walking;
        }
        else if (m_v_mean_magnitude >= m_running_threshold)
        {
            m_stance = Running;
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

/** \brief Returns the index of the largest magnitude element in a 3-array of doubles. Think np.argmax().
 * \param arr Vector of e.g. 3-velocity, 3-acceleration, etc.
 * \return The index of the element with largest value.
 */
StanceDetector::AXIS StanceDetector::biggest_axis(const std::array<double, 3> &arr)
{
    // TODO This should be using absolute value, since large negative values are 'bigger' than small positive values.
    AXIS axis;
    if (arr[0] > arr[1] && arr[0] > arr[2])
    {
        axis = XAxis;
    }
    else if (arr[1] > arr[0] && arr[1] > arr[2])
    {
        axis = YAxis;
    }
    else
    {
        axis = ZAxis;
    }
    return axis;
}

/** \brief Gives a measure of intensity of activity based on values of a, g, v.
 * \param a Acceleration magnitude.
 * \param g Angular velocity magnitude.
 * \param v Velocity magnitude.
 * \return Measure of intensity of activity.
 */
double StanceDetector::activity(double a, double g, double v)
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

/** \brief Calculates the mean of a vector of doubles.
 * \param values Vector of doubles.
 * \return Mean value of input vector.
 */
double StanceDetector::vector_mean(const std::vector<double> &values)
{
    double mean = 0;
    for (unsigned int i = 0; i < values.size(); i++)
    {
        mean += values[i];
    }
    return mean/values.size(); 
}

/** \brief Calculates the mean magnitude of a (x,3) sized vector of arrays of doubles.
 * Calculates the magnitude of each 3-array, then computes the mean of those magnitudes.
 * \param buffer Pointer to data buffer.
 * \return Mean magnitude as double.
 */
double StanceDetector::buffer_mean_magnitude(const std::vector<std::array<double, 3>> &buffer)
{
    double mean = 0.0;
    for (unsigned int i=0; i < buffer.size(); i++)
    {
        double square_sum = 0;
        for (unsigned int j=0; j < 3; j++)
        {
            square_sum += buffer[i][j] * buffer[i][j];
        }
        mean += sqrt(square_sum);
    }
    mean /= buffer.size();
    return mean;
}

/** \brief Calculates the mean magnitude of a (x,3) sized deque of arrays of doubles.
 * Calculates the magnitude of each 3-array, then computes the mean of those magnitudes.
 * \param buffer Pointer to data buffer.
 * \return Mean magnitude as double.
 */
double StanceDetector::buffer_mean_magnitude(const std::deque<std::array<double, 3>> &buffer)
{
    double mean = 0.0;
    for (unsigned int i=0; i < buffer.size(); i++)
    {
        double square_sum = 0;
        for (unsigned int j=0; j < 3; j++)
        {
            square_sum += buffer[i][j] * buffer[i][j];
        }
        mean += sqrt(square_sum);
    }
    mean /= buffer.size();
    return mean;
}

/** \brief Return the column-wise means of a size (x, 3) vector.
 * \param source_vector Pointer to source array.
 */
std::array<double, 3> StanceDetector::get_means(const std::vector<std::array<double, 3>> &source_vector)
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

/** \brief Gets the column-wise means of a deq<array<double>>.
 * \param source_vector Pointer to source array.
 * \return A 3-array containing the means.
 */
std::array<double, 3> StanceDetector::get_means(const std::deque<std::array<double, 3>> &source_vector)
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

/** \brief Gets the column-wise means of a deq<array<double>>.
 * \param source_vector Pointer to source array.
 * \return A 6-array containing the means.
 */
std::array<double, 6> StanceDetector::get_means(const std::deque<std::array<double, 6>> &source_vector)
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

// Getters ----------------------------------------------------------------------------------------

/** \brief Returns string representing current stance.
 * \return Current stance.
 */
StanceDetector::STANCE StanceDetector::getStance()
{
    stance_lock.lock();
    STANCE ret = m_stance;
    stance_lock.unlock();
    return ret;
}

/** \brief Gets the horizontal flag.
 * \return Integer bool.
 */
StanceDetector::ATTITUDE StanceDetector::getAttitude()
{  
    stance_lock.lock();
    ATTITUDE ret = m_attitude;
    stance_lock.unlock();
    return ret;
}

/** \brief Check if entangled flag set.
 * \return Integer bool.
 */
StanceDetector::ENTANGLED StanceDetector::getEntangledStatus()
{
    fall_lock.lock();
    ENTANGLED ret = m_entangled;
    m_entangled = NotEntangled;
    fall_lock.unlock();
    return ret;
}

/** \brief Get current fall flag.
 * \return Integer bool.
 */
StanceDetector::FALLING StanceDetector::getFallingStatus()
{
    fall_lock.lock();
    FALLING ret = m_falling;
    m_falling = NotFalling;
    fall_lock.unlock();
    return ret;
}
