#include <iostream>
#include <mutex>
#include <chrono>
#include <vector>
#include <thread>
#include <math.h>
#include <array>
#include <deque>
#include <fstream>
#include <string>

#include "stance.hpp"
#include "logger.hpp"
#include "vector3.hpp"
#include "arwain.hpp"

/** \brief Stance detection thread, periodically assesses mode of motion based in IMU and velocity data.
 * Runs as a thread.
 */
void stance_detector()
{
    // A little presleep to give IMU data a chance to collect and orientation filter chance to converge.
    std::this_thread::sleep_for(std::chrono::milliseconds{3000});

    // Stance detector object.
    arwain::StanceDetector stance{
        arwain::config.freefall_sensitivity,
        arwain::config.crawling_threshold,
        arwain::config.running_threshold,
        arwain::config.walking_threshold,
        arwain::config.active_threshold,
        arwain::config.struggle_threshold
    };

    // Local buffers.
    std::deque<Vector6> imu_data;
    std::deque<Vector3> vel_data;
    Quaternion rotation_quaternion;

    // Open file for freefall/entanglement logging
    arwain::Logger freefall_file;
    if (arwain::config.log_to_file)
    {
        freefall_file.open(arwain::folder_date_string + "/freefall.txt");
        freefall_file << "time freefall entanglement" << "\n";
    }

    // File handle for stance logging.
    arwain::Logger stance_file;
    if (arwain::config.log_to_file)
    {
        stance_file.open(arwain::folder_date_string + "/stance.txt");
        stance_file << "time stance" << "\n";
    }

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{arwain::Intervals::STANCE_DETECTION_INTERVAL};

    while (!arwain::shutdown)
    {
        while (arwain::system_mode == arwain::OperatingMode::Inference)
        {
            // Get all relevant data.
            { // TODO I just noticed that this is device IMU and not world IMU, and can't remember if that was intentional.
                std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
                imu_data = arwain::Buffers::IMU_BUFFER;
            }
            {
                std::lock_guard<std::mutex> lock{arwain::Locks::VELOCITY_BUFFER_LOCK};
                vel_data = arwain::Buffers::VELOCITY_BUFFER;
            }
            {
                std::lock_guard<std::mutex> lock{arwain::Locks::ORIENTATION_BUFFER_LOCK};
                rotation_quaternion = arwain::Buffers::QUAT_ORIENTATION_BUFFER.back();
            }

            // Update stance detector and get output. This can turn on but cannot turn off the falling and entangled flags.
            stance.update_attitude(rotation_quaternion);
            stance.run(imu_data, vel_data);
            arwain::status.current_stance = stance.getStance();
            arwain::status.falling = arwain::status.falling | stance.getFallingStatus();
            arwain::status.entangled = arwain::status.entangled | stance.getEntangledStatus();
            arwain::status.attitude = stance.getAttitude();

            // Log to file.
            if (arwain::config.log_to_file)
            {
                freefall_file << time.time_since_epoch().count() << " " << stance.getFallingStatus() << " " << stance.getEntangledStatus() << "\n";
                stance_file << time.time_since_epoch().count() << " " << stance.getStance() << "\n";
            }

            // Wait until the next tick.
            time = time + interval;
            std::this_thread::sleep_until(time);
        }
        sleep_ms(10);
    }

    // Close files
    if (arwain::config.log_to_file)
    {
        freefall_file.close();
        stance_file.close();
    }
}

// Constructors -----------------------------------------------------------------------------------

/** \brief Constructor for the stance class.
 * \param freefall_sensitivity Acceleration magnitude below which a fall event is detected.
 * \param crawling_threshold Speed below which, if horizontal, stance is detected as crawling.
 * \param running_threshold Speed above which, if vertical, stance is detected as running.
 * \param walking_threshold Speed above which, if vertical, stance is detected as walking.
 * \param active_threshold Value of the internal activity metric above which an entanglement event is detected.
 */
arwain::StanceDetector::StanceDetector(double freefall_sensitivity, double crawling_threshold, double running_threshold, double walking_threshold, double active_threshold, double struggle_threshold)
{
    m_freefall_sensitivity = freefall_sensitivity;
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

/** \brief Updates the attitude of the device, i.e. determines whether the device is horizontal or vertical.
 * For the current hardware configuration (Pi + IMU), we should expect the z axis of the IMU to be
 * roughly horizontal when the device is worn and the wearer is standing. Therefore the angle between
 * the device z axis and the world z axis should be about 90 degrees when the device is vertical. If this
 * angle becomes less than 45 degrees, we infer that the z axes are approaching each other and therefore
 * the device is in a horizontal configuration.
 * 
 * Actually we use the dot product between the two axes rather than the angle, and this reduces to simply
 * the z component of the rotated device orientation, since other components are zero.
 * 
 * If this z component is less than 0.707 (cos 45 degrees), we infer the device and therefore wearer
 * to be vertical.
 * 
 * \param rotation_quaternion The current rotation quaternion of the device, as determined by some orientation filter.
 */
void arwain::StanceDetector::update_attitude(Quaternion rotation_quaternion)
{
    auto rotated_device_z_component = (rotation_quaternion * Quaternion{0, 0, 0, 1} * rotation_quaternion.conjugate()).z;

    if (abs(rotated_device_z_component) < 0.707)
    {
        m_attitude = Vertical;
    }
    else
    {
        m_attitude = Horizontal;
    }
}

/** \brief Run detection algorithms against provided sensor data.
 * \param imu_data Pointer to deq<arr<double>> containging acceleration and gyro data.
 * \param vel_data Pointer to deq<arr<double>> containing velocity data.
 */
void arwain::StanceDetector::run(const std::deque<Vector6> &imu_data, const std::deque<Vector3> &vel_data)
{
    // Crunch the numbers ...
    std::vector<Vector3> accel_data;
    std::vector<Vector3> gyro_data;
    for (int i = 0; i < 20; i++)
    {
        accel_data.push_back(Vector3{
            imu_data[i].acce.x, imu_data[i].acce.y, imu_data[i].acce.z
        });
        gyro_data.push_back(Vector3{
            imu_data[i].gyro.x, imu_data[i].gyro.y, imu_data[i].gyro.z
        });
    }
    m_a_mean_magnitude = buffer_mean_magnitude(accel_data);
    m_g_mean_magnitude = buffer_mean_magnitude(gyro_data);
    m_v_mean_magnitude = buffer_mean_magnitude(vel_data);
    m_accel_means = get_means(accel_data);
    m_speed_means = get_means(vel_data);
    m_speed_axis = biggest_axis(m_speed_means);
    m_a_twitch = abs(m_a_mean_magnitude - m_gravity);
    m_struggle = vector_mean(m_struggle_window);
    m_activity = activity(m_a_mean_magnitude, m_g_mean_magnitude, m_v_mean_magnitude);
    m_tmp_struggle = (m_a_twitch + m_g_mean_magnitude) / (m_v_mean_magnitude + m_sfactor);
    m_struggle_window[m_count] = m_tmp_struggle;

    // TODO The assumptions about climbing are obviously wrong. Get rid of this or fix it.
    // If the axis with the highest average speed is the same as the vertical axis, subject must be climbing.
    // If the speed on the vertical axis exceed the climbing threshold, the subject must be climibing.
    // {
    //     m_primary_axis = biggest_axis(m_accel_means);
    //     std::lock_guard<std::mutex> lock{m_stance_lock};
    //     if (m_speed_axis == m_primary_axis || m_speed_means[m_vertical_axis] > m_climbing_threshold)
    //     {
    //         m_climbing = 1;
    //     }
    //     else
    //     {
    //         m_climbing = 0;
    //     }
    // }

    // Detect falling.
    if (m_a_mean_magnitude < m_freefall_sensitivity)
    {
        std::lock_guard<std::mutex> lock{m_fall_lock};
        m_falling = Falling;
    }

    // Detect entanglement, implied by high IMU activity but low velocity.
    if (m_struggle > m_struggle_threshold)
    {
        std::lock_guard<std::mutex> lock{m_fall_lock};
        m_entangled = Entangled;
    }

    // Detect stance.
    // Horizontal and slow => inactive
    // Horizontal and fast => crawling
    // Vertical and slow with low activity => inactive
    // Vertical and slow with high activity => searching
    // Vertical and moderate speed => walking
    // Vertical and high speed => running
    {
        std::lock_guard<std::mutex> lock{m_stance_lock};
        if (m_attitude == Horizontal)
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
        else if (m_attitude == Vertical)
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
    }

    m_count = (m_count + 1) % 10;
}

/** \brief Returns the index of the largest magnitude element in a 3-array of doubles. Think np.argmax().
 * \param arr Vector of e.g. 3-velocity, 3-acceleration, etc.
 * \return The index of the element with largest value.
 */
arwain::StanceDetector::Axis arwain::StanceDetector::biggest_axis(const Vector3 &arr)
{
    // This should be using absolute value, since large negative values are 'bigger' than small positive values.
    Axis axis;
    double x = abs(arr.x), y = abs(arr.y), z = abs(arr.z);
    if (x > y && x > z)
    {
        axis = XAxis;
    }
    else if (x < y && y > z)
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
double arwain::StanceDetector::activity(double a, double g, double v)
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
double arwain::StanceDetector::vector_mean(const std::vector<double> &values)
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
double arwain::StanceDetector::buffer_mean_magnitude(const std::vector<Vector3> &buffer)
{
    double mean = 0.0;
    for (unsigned int i=0; i < buffer.size(); i++)
    {
        double square_sum = 0;
        
            square_sum += buffer[i].x * buffer[i].x;
            square_sum += buffer[i].y * buffer[i].y;
            square_sum += buffer[i].z * buffer[i].z;
    
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
double arwain::StanceDetector::buffer_mean_magnitude(const std::deque<Vector3> &buffer)
{
    double mean = 0.0;
    for (unsigned int i=0; i < buffer.size(); i++)
    {
        double square_sum = 0;

            square_sum += buffer[i].x * buffer[i].x;
            square_sum += buffer[i].y * buffer[i].y;
            square_sum += buffer[i].z * buffer[i].z;

        mean += sqrt(square_sum);
    }
    mean /= buffer.size();
    return mean;
}

/** \brief Return the column-wise means of a size (x, 3) vector.
 * \param source_vector Pointer to source array.
 */
Vector3 arwain::StanceDetector::get_means(const std::vector<Vector3> &source_vector)
{
    Vector3 ret;
    unsigned int length = source_vector.size();
    for (unsigned int i = 0; i < length; i++)
    {
        ret.x += abs(source_vector[i].x);
        ret.y += abs(source_vector[i].y);
        ret.z += abs(source_vector[i].z);
    }
    ret.x /= length;
    ret.y /= length;
    ret.z /= length;

    return ret;
}

/** \brief Gets the column-wise means of a deq<array<double>>.
 * \param source_vector Pointer to source array.
 * \return A 3-array containing the means.
 */
Vector3 arwain::StanceDetector::get_means(const std::deque<Vector3> &source_vector)
{
    Vector3 ret;
    unsigned int length = source_vector.size();
    for (unsigned int i = 0; i < length; i++)
    {
         ret.x += abs(source_vector[i].x);
        ret.y += abs(source_vector[i].y);
        ret.z += abs(source_vector[i].z);
    }
    ret.x /= length;
    ret.y /= length;
    ret.z /= length;

    return ret;
}

/** \brief Gets the column-wise means of a deq<array<double>>.
 * \param source_vector Pointer to source array.
 * \return A 6-array containing the means.
 */
std::array<double, 6> arwain::StanceDetector::get_means(const std::deque<std::array<double, 6>> &source_vector)
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
arwain::StanceDetector::Stance arwain::StanceDetector::getStance()
{
    std::lock_guard<std::mutex> lock{m_stance_lock};
    Stance ret = m_stance;
    return ret;
}

/** \brief Gets the horizontal flag.
 * \return Integer bool.
 */
arwain::StanceDetector::Attitude arwain::StanceDetector::getAttitude()
{  
    std::lock_guard<std::mutex> lock{m_stance_lock};
    Attitude ret = m_attitude;
    return ret;
}

/** \brief Check if entangled flag set.
 * \return Integer bool.
 */
arwain::StanceDetector::EntangleState arwain::StanceDetector::getEntangledStatus()
{
    std::lock_guard<std::mutex> lock{m_fall_lock};
    EntangleState ret = m_entangled;
    m_entangled = NotEntangled;
    return ret;
}

/** \brief Get current fall flag.
 * \return Integer bool.
 */
arwain::StanceDetector::FallState arwain::StanceDetector::getFallingStatus()
{
    std::lock_guard<std::mutex> lock{m_fall_lock};
    FallState ret = m_falling;
    m_falling = NotFalling;
    return ret;
}

/** \brief If either value indicates Falling status, return Falling status.
 * \param stance1 One of the stances to compare.
 * \param stance2 The other stance to compare.
 */
arwain::StanceDetector::FallState operator|(const arwain::StanceDetector::FallState &stance1, const arwain::StanceDetector::FallState &stance2)
{
    if (stance1 == arwain::StanceDetector::Falling || stance2 == arwain::StanceDetector::Falling)
    {
        return arwain::StanceDetector::Falling;
    }
    else
    {
        return arwain::StanceDetector::NotFalling;
    }
}

/** \brief If either value indicates Entangled status, return Entangled status.
 * \param stance1 One of the stances to compare.
 * \param stance2 The other stance to compare.
 */
arwain::StanceDetector::EntangleState operator|(const arwain::StanceDetector::EntangleState &stance1, const arwain::StanceDetector::EntangleState &stance2)
{
    if (stance1 == arwain::StanceDetector::Entangled || stance2 == arwain::StanceDetector::Entangled)
    {
        return arwain::StanceDetector::Entangled;
    }
    else
    {
        return arwain::StanceDetector::NotEntangled;
    }
}
