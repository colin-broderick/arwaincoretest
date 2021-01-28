#include "efaroe.h"


/** \brief Constructor for eFaroe class.
 * eFaroe is an orientation filter which uses acceleration and angular velocity measurements from accelerometer and gyroscope to attempt to track current orientation.
 * \param initial_quaternion The known initial orientation as a quaternion. Use {1, 0, 0, 0} if unknown.
 * \param gyro_bias The known constant bias in the gyroscope. Use zeroes if unknown.
 * \param gyro_error TODO not sure what this is for.
 * \param use_mag Whether to use magnetometer data for the orientation filter. TODO Not yet implemented so should always be zero.
 */
eFaroe::eFaroe(quaternion initial_quaternion, std::array<double, 3> gyro_bias, double gyro_error, int use_mag)
{
    gyro_bias = gyro_bias;
    gyro_error = 0.05;
    uk_dip = -67.0*3.14159265/180.0;
    emf = {cos(uk_dip), sin(uk_dip)};
    zeta = sqrt(3) * 1e-2;
    last_read = 0;
    use_mag = use_mag;
    
    if (initial_quaternion == quaternion{0, 0, 0, 0})
    {
        q = {1, 0, 0, 0};
        gyro_error = 100;
        true_error = 100;
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

/** \brief Update the internal state of the orientation filter by supplying acceleratometer and gyroscope values.
 *  \param timestamp Timestamp of the supplied data in nanoseconds.
 *  \param ax x-axis accelerometer value in m/s2
 *  \param ay y-axis accelerometer value in m/s2
 *  \param az x-axis accelerometer value in m/s2
 *  \param gx x-axis gyroscope value in rad/s
 *  \param gy y-axis gyroscope value in rad/s
 *  \param gz z-axis gyroscope value in rad/s
 *  \return Nothing; updates internal state.
 */
void eFaroe::update(double timestamp, double ax, double ay, double az, double gx, double gy, double gz)
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

    std::array<double, 3> acc = {ax, ay, az};
    std::array<double, 3> gyr = {gx, gy, gz};

    if (last_read == 0)
    {
        // Confirm timestamp has usable type/value
        last_read = timestamp/1e9;
        return;
    }
    else
    {
        dt = timestamp/1e9 - last_read;
        if (dt > 1)
        {
            dt = 1;
        }
        last_read = timestamp/1e9;
    }
    
    // Normalize acceleration.
    double a_norm = 1.0/sqrt(ax*ax+ay*ay+az*az);
    ax = ax*a_norm;
    ay = ay*a_norm;
    az = az*a_norm;
    
    // Construct Jacobian.
    std::array<double, 3> jac_a{
        q.x*2*q.z - q.w*2*q.y,
        q.w*2*q.x + q.y*2*q.z,
        1 - 2*q.x*q.x - 2*q.y*q.y
    };

    // Calculate gradient.
    std::array<double, 3> grad = cross(jac_a, acc);

    // Normalize gradient.
    double grad_norm = 1.0/sqrt(grad[0]*grad[0]+grad[1]*grad[1]+grad[2]*grad[2]);
    grad[0] = grad[0]*grad_norm;
    grad[1] = grad[0]*grad_norm;
    grad[2] = grad[0]*grad_norm;

    // TODO Calculate new gyro_bias?
    std::array<double, 3> g_b;
    g_b[0] = gyro_bias[0] + grad[0]*dt*zeta;
    g_b[1] = gyro_bias[1] + grad[1]*dt*zeta;
    g_b[2] = gyro_bias[2] + grad[2]*dt*zeta;

    // Subtract gyro bias.
    std::array<double, 3> gyro;
    gyro[0] = gyr[0] - g_b[0];
    gyro[1] = gyr[1] - g_b[1];
    gyro[2] = gyr[2] - g_b[2];
    
    // TODO What is this?
    std::array<double, 3> a_v;
    a_v[0] = dt*(gyro[0]-beta*grad[0]);
    a_v[1] = dt*(gyro[1]-beta*grad[1]);
    a_v[2] = dt*(gyro[2]-beta*grad[2]);
    quaternion qav{0, a_v[0], a_v[1], a_v[2]};

    // Calculate delta orientation quaternion.
    quaternion dq = q*qav*0.5;
    q = (q + dq).unit();

    computed_angles = 0;
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
