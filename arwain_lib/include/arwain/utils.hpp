#ifndef _ARWAIN_UTILS_HPP
#define _ARWAIN_UTILS_HPP

#include <chrono>
#include <thread>
#include <deque>
#include <array>
#include <vector>

void sleep_ms(int ms);

class Quaternion;
class Vector3;

double unwrap_phase_degrees(double new_angle, const double previous_angle);
double unwrap_phase_radians(double new_angle, const double previous_angle);
std::string date_time_string();
std::array<double, 3> vec_to_array3(std::vector<double> in_vector);
void pin_thread(std::jthread& th, int core_number);

/** \brief Clamps a value between a min and max.
 * \param value The value to be clamped.
 * \param minimum The minimum the value is allowed to be.
 * \param maximum The maximum the value is allowed to be.
 * \return value < minimum ? minimum : (value > maximum ? maximum : value).
*/
template <typename T>
constexpr int clamp_value(const T value, const T minimum, const T maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    else if (value > maximum)
    {
        return maximum;
    }
    else
    {
        return value;
    }
}

struct EulerOrientation
{
    double roll;
    double pitch;
    double yaw;
};

namespace arwain
{
    EulerOrientation compute_euler(Quaternion& quaternion_rotor);
    Vector3 apply_quat_rotor_to_vector3(const Vector3& vector, const Quaternion& quaternion);

    enum class ReturnCode
    {
        Success = 0,
        FailedIMU = -1,
        FailedConfiguration = -2,
        FailedMagnetometer = -3,
        NoConfigurationFile = -4,
        NoInferenceFile = -5,
        IMUReadError = -6,
        IOError = -7,
        PressureReadError = -8,
        MagnetometerReadError = -9,
        InferenceError = -10,
        GeneralError = -11
    };
    
    /** \brief Compute the average value of an indeterminiate number of objects. */
    template <class T, class... Args>
    T average(T first, Args... args)
    {
        double count = 1.0;
        for (const auto& arg : {args...})
        {
            first = first + arg;
            count += 1.0;
        }
        return first / count;
    }
}

#endif
