#include <cmath>
#include <sstream>

#include <arwain/quaternion.hpp>
#include <arwain/vector3.hpp>

#include "arwain/utils.hpp"

void sleep_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds{ms});
}

/** \brief Converts a rotation quaternion to Euler angles, i.e. roll, pitch, and yaw.
 * \note The Euler angle representation should never be used for calculations; they are for display
 * and logging only.
 * \param quaternion_rotor A quaternion of magnitude one representing a rotation.
 * \return The Euler angle (roll, pitch, yaw) representation of the rotation achieved by the quaternion.
*/
EulerOrientation arwain::compute_euler(Quaternion& quaternion_rotor)
{
    EulerOrientation euler;
    euler.roll = std::atan2(quaternion_rotor.w*quaternion_rotor.x + quaternion_rotor.y*quaternion_rotor.z, 0.5 - quaternion_rotor.x*quaternion_rotor.x - quaternion_rotor.y*quaternion_rotor.y);
    euler.pitch = std::asin(-2.0 * (quaternion_rotor.x*quaternion_rotor.z - quaternion_rotor.w*quaternion_rotor.y));
    euler.yaw = std::atan2(quaternion_rotor.x*quaternion_rotor.y + quaternion_rotor.w*quaternion_rotor.z, 0.5 - quaternion_rotor.y*quaternion_rotor.y - quaternion_rotor.z*quaternion_rotor.z);
    return euler;
}

/** \brief Applies the quaternion to the vector as in:
 *      q * v * q.inverse
 * \param vector Vector3 to be rotated by the quaternion.
 * \param quaternion Quaternion form of the rotation operation to be applied.
 * \return The Vector3 result of the rotation of vector by quaternion.
*/
Vector3 arwain::apply_quat_rotor_to_vector3(const Vector3& vector, const Quaternion& quaternion)
{
    // Convert the 3-vector into a quaternion.
    Quaternion quat_vec{0, vector.x, vector.y, vector.z};

    // Compute the rotated vector as a quaternion.
    Quaternion rotated_quaternion_vector = quaternion * quat_vec * quaternion.conjugate();

    // Cast the rotated quaternion back into a 3-vector.
    return Vector3{
        rotated_quaternion_vector.x,
        rotated_quaternion_vector.y,
        rotated_quaternion_vector.z
    };
}

/** \brief Clamps a value between a min and max.
 * \param value The value to be clamped.
 * \param minimum The minimum the value is allowed to be.
 * \param maximum The maximum the value is allowed to be.
 * \return value < minimum ? minimum : (value > maximum ? maximum : value).
*/
int clamp_value(int value, int minimum, int maximum)
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

double unwrap_phase_radians(double new_angle, double previous_angle)
{
    while (new_angle - previous_angle > 3.14159)
    {
        new_angle -= 2.0 * 3.14159;
    }
    while (new_angle - previous_angle < -3.14159)
    {
        new_angle += 2.0 * 3.14159;
    }
    return new_angle;
}

double unwrap_phase_degrees(double new_angle, double previous_angle)
{
    while (new_angle - previous_angle > 180.0)
    {
        new_angle -= 360.0;
    }
    while (new_angle - previous_angle < -180.0)
    {
        new_angle += 360.0;
    }
    return new_angle;
}

/** \brief Get the current system datetime as a string.
 * \return Datetime as string in the format yyyy_mm_dd_hh_mm_ss.
 */
std::string date_time_string()
{
    std::time_t now = std::time(0);
    std::tm *ltm = localtime(&now);
    std::stringstream ss;

    // Year
    ss << ltm->tm_year+1900;
    ss << "_";

    // Month
    if (ltm->tm_mon+1 < 10)
    {
        ss << '0' << ltm->tm_mon+1;
    }
    else
    {
        ss << ltm->tm_mon+1;
    }
    ss << "_";

    // Date
    if (ltm->tm_mday < 10)
    {
        ss << '0' << ltm->tm_mday;
    }
    else
    {
        ss << ltm->tm_mday;
    }
    ss << "_";
    
    // Hour
    if (ltm->tm_hour < 10)
    {
        ss << '0' << ltm->tm_hour;
    }
    else
    {
        ss << ltm->tm_hour;
    }
    ss << "_";

    // Minute
    if (ltm->tm_min < 10)
    {
        ss << '0' << ltm->tm_min;
    }
    else
    {
        ss << ltm->tm_min;
    }
    ss << "_";

    // Second
    if (ltm->tm_sec < 10)
    {
        ss << '0' << ltm->tm_sec;
    }
    else
    {
        ss << ltm->tm_sec;
    }

    return ss.str();
}
