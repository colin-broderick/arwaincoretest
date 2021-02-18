#include "efaroe.h"

/** \brief Constructor for eFaroe class.
 * eFaroe is an orientation filter which uses acceleration and angular velocity measurements from accelerometer and gyroscope to attempt to track current orientation.
 * \param initial_quaternion The known initial orientation as a quaternion. Use {1, 0, 0, 0} if unknown.
 * \param gyro_bias The known constant bias in the gyroscope. Use zeroes if unknown.
 * \param gyro_error TODO not sure what this is for.
 * \param use_mag Whether to use magnetometer data for the orientation filter. TODO Not yet implemented so should always be zero.
 */
eFaroe::eFaroe(quaternion initial_quaternion, vector3 gyro_bias, double gyro_error, int use_mag)
{
    gyro_bias = gyro_bias;
    gyro_error = 0.05;
    uk_dip = -67.0*3.14159265/180.0;
    emf = {cos(uk_dip), 0, sin(uk_dip)};
    zeta = sqrt(3) * 1e-2;
    last_read = 0;
    use_mag = use_mag;
    
    if (initial_quaternion == quaternion{0, 0, 0, 0})
    {
        q = {1, 0, 0, 0};
        true_error = gyro_error;
        gyro_error = 100;
        beta = sqrt(3) * 100;
        conv_count = 100;
    }
    else
    {
        q = initial_quaternion;
        gyro_error = gyro_error;
        true_error = gyro_error;
        beta = sqrt(3) * gyro_error;
        conv_count = -1;
    }
}

/** \brief Update the internal state of the orientation filter by supplying accelerometer and gyroscope values.
 *  \param timestamp Timestamp of the supplied data in nanoseconds.
 *  \param gx x-axis gyroscope value in rad/s
 *  \param gy y-axis gyroscope value in rad/s
 *  \param gz z-axis gyroscope value in rad/s
 *  \param ax x-axis accelerometer value in m/s2
 *  \param ay y-axis accelerometer value in m/s2
 *  \param az x-axis accelerometer value in m/s2
 *  \return Nothing; updates internal state.
 */
void eFaroe::updateIMU(double timestamp, double gx, double gy, double gz,  double ax, double ay, double az)
{
    if (conv_count > 0)
    {
        conv_count--;
        if (conv_count == 0)
        {
            std::cout << "efaroe converged" << "\n";
            gyro_error = true_error;
            beta = sqrt(3) * gyro_error;
        }
    }

    vector3 acc{ax, ay, az};
    vector3 gyr{gx, gy, gz};

    // Convert timestamp to seconds.
    timestamp = timestamp/1e9;
    if (last_read == 0)
    {
        last_read = timestamp;
        return;
    }
    else
    {
        dt = timestamp - last_read;
        if (dt > 1)
        {
            dt = 1;
        }
        last_read = timestamp;
    }
    
    // Normalize acceleration.
    acc = acc/acc.magnitude();
    
    // Construct Jacobian.
    vector3 jacobian_a{
        q.x*2*q.z - q.w*2*q.y,
        q.w*2*q.x + q.y*2*q.z,
        1 - 2*q.x*q.x - 2*q.y*q.y
    };

    // Calculate gradient.
    vector3 gradient = cross(jacobian_a, acc);

    // Normalize gradient.
    gradient = gradient/gradient.magnitude();

    // Calculate new gyro_bias?
    vector3 g_b = gyro_bias + gradient * dt * zeta;

    // Subtract gyro bias.
    vector3 gyro = gyr - g_b;

    // EXPERIMENTAL Gyro bias filter.
    // gyro_bias = gyro_bias + gyro*0.0001;

    // TODO What is this?
    vector3 a_v = (gyro - gradient*beta)*dt;
    quaternion qav{a_v};

    // Calculate delta orientation quaternion.
    quaternion dq = q*qav*0.5;
    q = (q + dq).unit();

    computed_angles = 0;
}

/** \brief Update the internal state of the orientation filter using both IMU and magnetometer readings.
 * \param timestamp Timestamp of the supplied data in nanoseconds.
 * \param gx x-axis gyroscope value in rad/s
 * \param gy y-axis gyroscope value in rad/s
 * \param gz z-axis gyroscope value in rad/s
 * \param ax x-axis accelerometer value in m/s2
 * \param ay y-axis accelerometer value in m/s2
 * \param az x-axis accelerometer value in m/s2
 * \return Nothing; updates internal state.
 */
void eFaroe::updateIMU(double timestamp, double gx, double gy, double gz,  double ax, double ay, double az, double mx, double my, double mz)
{
    if (conv_count > 0)
    {
        conv_count--;
        if (conv_count == 0)
        {
            std::cout << "efaroe converged" << "\n";
            gyro_error = true_error;
            beta = sqrt(3) * gyro_error;
        }
    }

    vector3 acc{ax, ay, az};
    vector3 gyr{gx, gy, gz};
    vector3 mag{mx, my, mz};

    // Convert timestamp to seconds.
    timestamp = timestamp/1e9;
    if (last_read == 0)
    {
        last_read = timestamp;
        return;
    }
    else
    {
        dt = timestamp - last_read;
        if (dt > 1)
        {
            dt = 1;
        }
        last_read = timestamp;
    }

    acc = acc/acc.magnitude();
    mag = mag/mag.magnitude();

    mag = cross(acc, mag);
    vector3 r_emf = cross(vector3{0,0,1}, emf);

    // Construct acceleration Jacobian.
    vector3 jacobian_a{
        q.x*2*q.z - q.w*2*q.y,
        q.w*2*q.x + q.y*2*q.z,
        1 - 2*q.x*q.x - 2*q.y*q.y
    };

    // Construct magnetic Jacobian.
    vector3 jacobian_m{
        2 * ((r_emf.x * (0.5 - q.y*q.y - q.z*q.z))   + (r_emf.z * q.x * q.z)),
        2 * ((r_emf.x * ((q.x * q.y) - (q.w * q.z))) + (r_emf.z * ((q.w * q.z) + (q.x * q.y)))),
        2 * (r_emf.x * ((q.w * q.y) + (q.x * q.z))   + (r_emf.z * (0.5 - q.x*q.x - q.y*q.y)))
    };

    // Calculate gradient
    vector3 gradient = cross(jacobian_a, acc) + cross(jacobian_m, mag);

    // Normalize gradient
    gradient = gradient/gradient.magnitude();

    // Calculate new gyro_bias?
    vector3 g_b = gyro_bias + gradient * dt * zeta;

    // Subtract gyro bias.
    vector3 gyro = gyr - g_b;

    // EXPERIMENTAL Gyro bias filter.
    // gyro_bias = gyro_bias + gyro*0.0001;

    // TODO What is this?
    vector3 a_v = (gyro - gradient*beta)*dt;
    quaternion qav{a_v};

    // Calculate delta orientation quaternion.
    quaternion dq = q*qav*0.5;
    q = (q + dq).unit();

    // TODO I have basically no idea what this block is doing
    {
        local_emf_test = {
            mag.x * (0.5 - q.y*q.y - q.z*q.z)   + mag.y * ((q.x * q.y) - (q.w * q.z)) + mag.z * ((q.y * q.z) + (q.w * q.y)),
            mag.x * ((q.x * q.y) + (q.w * q.z)) + mag.y * (0.5 - q.x*q.x - q.z*q.z)   + mag.z * ((q.y * q.z) - (q.w * q.x)),
            mag.x * ((q.y * q.z) - (q.w * q.y)) + mag.y * ((q.y * q.z) + (q.w * q.x)) + mag.z * (0.5 - q.x*q.x - q.y*q.y)
        };

        emf_x_test = vector3{local_emf_test.x, local_emf_test.y, 0}.magnitude();
        emf = {emf_x_test, 0, local_emf_test.z};
    }


}

/** \brief Get the real component of the orientation quaternion.
 * \return Real quaternion component.
 */
double eFaroe::getW()
{
    return q.w;
}

/** \brief Get the i/x component of the orientation quaternion.
 * \return i/x component of the orientation quaternion.
 */
double eFaroe::getX()
{
    return q.x;
}

/** \brief Get the j/y component of the orientation quaternion.
 * \return j/y component of the orientation quaternion.
 */
double eFaroe::getY()
{
    return q.y;
}

/** \brief Get the k/z component of the orientation quaternion.
 * \return k/z component of the orientation quaternion.
 */
double eFaroe::getZ()
{
    return q.z;
}

/** \brief Get the pitch Euler angle.
 * \return Pitch Euler angle.
 */
double eFaroe::getPitch()
{
    if (!computed_angles)
    {
        computeAngles();
    }
    return pitch * degrees_per_radian;
}

/** \brief Get the yaw Euler angle.
 * \return Yaw Euler angle.
 */
double eFaroe::getYaw()
{
    if (!computed_angles)
    {
        computeAngles();
    }
    return yaw * degrees_per_radian + 180.0;
}

/** \brief Get the roll Euler angle.
 * \return Roll Euler angle.
 */
double eFaroe::getRoll()
{
    if (!computed_angles)
    {
        computeAngles();
    }
    return roll * degrees_per_radian;
}

/** \brief Get the full orientation quaternion.
 * \return Full orientation quaternion.
 */
quaternion eFaroe::getQuat()
{
    return q;
}

/** \brief Update internally stored Euler angles in degrees.
 * For performance reasons these angles are only updated when they are requested.
 * \return Nothing; updates internal state.
 */
void eFaroe::computeAngles()
{
    roll = atan2f(q.w*q.x + q.y*q.z, 0.5f - q.x*q.x - q.y*q.y);
    pitch = asinf(-2.0f * (q.x*q.z - q.w*q.y));
    yaw = atan2f(q.x*q.y + q.w*q.z, 0.5f - q.y*q.y - q.z*q.z);
}

/** \brief Retrieve the orientation in the Euler angle representation.
 * If the internally stored angles are not up to date, they will be updated before returning..
 * \return Euler angles in degrees, in the order roll, pitch, yaw.
 */
std::array<double, 3> eFaroe::getEuler()
{
    if (!computed_angles)
    {
        computeAngles();
    }
    return std::array<double, 3>{roll, pitch, yaw};
}
