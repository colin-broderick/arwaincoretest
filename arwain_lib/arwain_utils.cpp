#include <cmath>

#include "arwain_utils.hpp"
#include "quaternion.hpp"
#include "vector3.hpp"

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

RollingAverage::RollingAverage(unsigned int window_size_)
: window_size(window_size_)
{

}

bool RollingAverage::ready()
{
    return stack.size() == window_size;
}

void RollingAverage::feed(double value)
{
    current_average += value;
    stack.push_back(value);
    if (stack.size() > window_size)
    {
        current_average -= stack.front() ;
        stack.pop_front();
    }
}

double RollingAverage::get_value()
{
    return current_average / static_cast<double>(window_size);
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
