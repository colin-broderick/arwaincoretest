#include "efaroe.hpp"
 
/** \brief Constructor for eFaroe class.
 * eFaroe is an orientation filter which uses acceleration and angular velocity measurements from accelerometer and gyroscope to attempt to track current orientation.
 * \param initial_quaternion The known initial orientation as a quaternion. Use {1, 0, 0, 0} if unknown.
 * \param gyro_bias The known constant bias in the gyroscope. Use zeroes if unknown.
 * \param gyro_error TODO not sure what this is for.
 * TODO Make use of beta being passed in.
 */
arwain::eFaroe::eFaroe(quaternion initial_quaternion, vector3 gyro_bias, double gyro_error, double beta, double zeta)
{
    m_zeta = zeta;
    m_gyro_bias = gyro_bias;
    gyro_error = 0.05;
    uk_dip = -67.0*3.14159265/180.0;
    emf = {cos(uk_dip), 0, sin(uk_dip)};
    last_read = 0;
    m_true_error = gyro_error;
    
    if (initial_quaternion == quaternion{0, 0, 0, 0})
    {
        m_quaternion = {1, 0, 0, 0};
        m_gyro_error = 100;
        conv_count = 100;
    }
    else
    {
        m_quaternion = initial_quaternion;
        m_gyro_error = gyro_error;
        conv_count = -1;
    }

    m_beta = sqrt(3) * m_gyro_error;
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
void arwain::eFaroe::update(double timestamp, double gx, double gy, double gz,  double ax, double ay, double az)
{
    // Alias m_quaternion for readability.
    quaternion& q = m_quaternion;
    
    if (conv_count > 0)
    {
        conv_count--;
        if (conv_count == 0)
        {
            std::cout << "efaroe converged" << "\n";
            m_gyro_error = m_true_error;
            m_beta = sqrt(3) * m_gyro_error;
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
    vector3 g_b = m_gyro_bias + gradient * dt * m_zeta;

    // Subtract gyro bias.
    vector3 gyro = gyr - g_b;

    // EXPERIMENTAL Gyro bias filter.
    // gyro_bias = gyro_bias + gyro*0.0001;

    // TODO What is this?
    vector3 a_v = (gyro - gradient * m_beta) * dt;
    quaternion qav{a_v.x, a_v.y, a_v.z};

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
void arwain::eFaroe::update(double timestamp, double gx, double gy, double gz,  double ax, double ay, double az, double mx, double my, double mz)
{
    // Alias m_quaternion for readability.
    quaternion& q = m_quaternion;

    if (conv_count > 0)
    {
        conv_count--;
        if (conv_count == 0)
        {
            std::cout << "efaroe converged" << "\n";
            m_gyro_error = m_true_error;
            m_beta = sqrt(3) * m_gyro_error;
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
    vector3 g_b = m_gyro_bias + gradient * dt * m_zeta;

    // Subtract gyro bias.
    vector3 gyro = gyr - g_b;

    // EXPERIMENTAL Gyro bias filter.
    // gyro_bias = gyro_bias + gyro*0.0001;

    // TODO What is this?
    vector3 a_v = (gyro - gradient * m_beta) * dt;
    quaternion qav{a_v.x, a_v.y, a_v.z};

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
double arwain::eFaroe::getW()
{
    return m_quaternion.w;
}

/** \brief Get the i/x component of the orientation quaternion.
 * \return i/x component of the orientation quaternion.
 */
double arwain::eFaroe::getX()
{
    return m_quaternion.x;
}

/** \brief Get the j/y component of the orientation quaternion.
 * \return j/y component of the orientation quaternion.
 */
double arwain::eFaroe::getY()
{
    return m_quaternion.y;
}

/** \brief Get the k/z component of the orientation quaternion.
 * \return k/z component of the orientation quaternion.
 */
double arwain::eFaroe::getZ()
{
    return m_quaternion.z;
}

/** \brief Get the pitch Euler angle.
 * \return Pitch Euler angle.
 */
double arwain::eFaroe::getPitch()
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
double arwain::eFaroe::getYaw()
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
double arwain::eFaroe::getRoll()
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
quaternion arwain::eFaroe::getQuat()
{
    return m_quaternion;
}

/** \brief Update internally stored Euler angles in degrees.
 * For performance reasons these angles are only updated when they are requested.
 * \return Nothing; updates internal state.
 */
void arwain::eFaroe::computeAngles()
{
    quaternion& q = m_quaternion;
    roll = atan2f(q.w*q.x + q.y*q.z, 0.5f - q.x*q.x - q.y*q.y);
    pitch = asinf(-2.0f * (q.x*q.z - q.w*q.y));
    yaw = atan2f(q.x*q.y + q.w*q.z, 0.5f - q.y*q.y - q.z*q.z);
}

/** \brief Retrieve the orientation in the Euler angle representation.
 * If the internally stored angles are not up to date, they will be updated before returning..
 * \return Euler angles in degrees, in the order roll, pitch, yaw.
 */
std::array<double, 3> arwain::eFaroe::getEuler()
{
    if (!computed_angles)
    {
        computeAngles();
    }
    return std::array<double, 3>{roll, pitch, yaw};
}
