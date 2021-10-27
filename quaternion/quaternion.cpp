#include <cmath>

#include "quaternion.hpp"

/** \brief Computes a weighted 'average' of two quaternions by SLERP (or LERP if applicable).
 * \param quat1 The first quaternion. This one cannot be a const reference since we might need to modify it.
 * \param quat2 The second quaterion.
 * \param t The interpolation factor. Between 0 and 1. Low numbers favour quat1, high numbers favour quat2.
 * \return A new quaternion.
 */
quaternion quaternion::slerp(quaternion quat1, const quaternion& quat2, const double t)
{
    // TODO Check for valid lengths or other things that should generate errors.
    double cosHalfAngle = quat1.w*quat2.w + quat1.x*quat2.x + quat1.y*quat2.y + quat1.z*quat2.z;

    // If half angle is zero, return whichever
    if (cosHalfAngle <= -1.0 || cosHalfAngle >= 1.0)
    {
        return quat1;
    }

    // If coshalfangle < 0, invert one of the quaternions
    if (cosHalfAngle < 0.0)
    {
        quat1 = -quat1;
        cosHalfAngle = -cosHalfAngle;
    }

    // If the angle is large, do SLERP. If the angle is very small, LERP will suffice.
    double blendA, blendB;
    if (cosHalfAngle < 0.99)
    {
        double halfAngle = std::acos(cosHalfAngle);
        double invSinHalfAngle = 1.0 / std::sin(halfAngle);
        blendA = std::sin(halfAngle * (1.0 - t)) * invSinHalfAngle;
        blendB = std::sin(halfAngle * t) * invSinHalfAngle;
    }
    else
    {
        blendA = 1.0 - t;
        blendB = t;
    }

    return (blendA * quat1 + blendB * quat2).unit();
}

// Constructors ===================================================================================

quaternion::quaternion() : quaternion(1, 0, 0, 0)
{
}

quaternion::quaternion(const std::array<double, 3>& vec) : quaternion(0, vec[0], vec[1], vec[2])
{
}

/** \brief Construct a rotation from the axis-angle representation.
 * \param angle The angle in radians by which to rotate the frame.
 * \param axis The axis around which to rotate the frame.
 */
quaternion::quaternion(const double angle, const std::array<double, 3>& axis)
{
    double axisInvNorm = 1.0/std::sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    this->angle = angle;
    this->axis = axis;
    this->w = std::cos(angle/2.0);
    this->x = std::sin(angle/2.0) * axis[0] * axisInvNorm;
    this->y = std::sin(angle/2.0) * axis[1] * axisInvNorm;
    this->z = std::sin(angle/2.0) * axis[2] * axisInvNorm;
}

/** \brief Construct a quaternion.
 * \param w The real component of the quaternion.
 * \param x The i component of the quaternion.
 * \param y The j component of the quaternion.
 * \param z The k component of the quaternion.
 */
quaternion::quaternion(const double w, const double x, const double y, const double z)
{
    this->w = w;
    this->x = x;
    this->y = y;
    this->z = z;
    this->angle = 2 * std::acos(w);
    this->axis[0] = x / std::sqrt(1 - w*w);
    this->axis[1] = y / std::sqrt(1 - w*w);
    this->axis[2] = z / std::sqrt(1 - w*w);
}

// Static methods =================================================================================

/** \brief Compute the inner product of two quaternions.
 * \param quat1 The first quaternion.
 * \param quat2 The second quaternion.
 * \return Scalar inner product of the two quaternions.
 */
double quaternion::dotProduct(const quaternion& quat1, const quaternion& quat2)
{
    return quat1.getW() * quat2.getW()
         + quat1.getX() * quat2.getX()
         + quat1.getY() * quat2.getY()
         + quat1.getZ() * quat2.getZ();
}

// Operators ======================================================================================

/** \brief Arithmetic sum of two quaternions.
 * \param quat1 The quaternion from which to subtract.
 * \param quat2 The quaternion to subtract.
 * \return The new quaternion after computing the difference.
 */
quaternion operator+(const quaternion& quat1, const quaternion& quat2)
{
    return quaternion{
        quat1.getW() + quat2.getW(),
        quat1.getX() + quat2.getX(),
        quat1.getY() + quat2.getY(),
        quat1.getZ() + quat2.getZ()
    };
}

/** \brief Arithmetic difference of two quaternions.
 * \param quat1 The first quaternion in the sum.
 * \param quat2 The second quaternion in the sum.
 * \return The new quaternion after computing the sum.
 */
quaternion operator-(const quaternion& quat1, const quaternion& quat2)
{
    return quaternion{
        quat1.getW() - quat2.getW(),
        quat1.getX() - quat2.getX(),
        quat1.getY() - quat2.getY(),
        quat1.getZ() - quat2.getZ()
    };
}

/** \brief Multiply two quaternions.
 * \param quat1 The first quaternion in the product.
 * \param quat2 The second quaternion in the product.
 * \return The new quaternion after computing the product.
 */
quaternion operator*(const quaternion& quat1, const quaternion& quat2)
{
    double p1 = quat1.getW();
    double p2 = quat1.getX();
    double p3 = quat1.getY();
    double p4 = quat1.getZ();

    double q1 = quat2.getW();
    double q2 = quat2.getX();
    double q3 = quat2.getY();
    double q4 = quat2.getZ();

    return quaternion{
        p1*q1 - p2*q2 - p3*q3 - p4*q4,
        p1*q2 + p2*q1 + p3*q4 - p4*q3,
        p1*q3 - p2*q4 + p3*q1 + p4*q2,
        p1*q4 + p2*q3 - p3*q2 + p4*q1
    };
}

/** \brief Negate a quaternion, i.e. multiply by -1. */
quaternion operator-(const quaternion& quaternion)
{
    return -1 * quaternion;
}

/** \brief Two quaternions are equal if all their elements are equal.
 * \param quat1 The first quaternion.
 * \param quat2 The second quaternion.
 * \return Bool indicating equality.
 */
bool operator==(const quaternion& quat1, const quaternion& quat2)
{
    if (quat1.getW() != quat2.getW())
    {
        return false;
    }
    if (quat1.getX() != quat2.getX())
    {
        return false;
    }
    if (quat1.getY() != quat2.getY())
    {
        return false;
    }
    if (quat1.getZ() != quat2.getZ())
    {
        return false;
    }
    return true;
}

/** \brief quaternion-stream insertion operator, for printing or writing to file.
 * \param stream The ofstream to write the quaternion to.
 * \param quaternion A quaternion to print or write to file.
 * \return A reference to the output stream.
 */
std::ostream& operator<<(std::ostream& stream, const quaternion& quaternion)
{
    stream << "quaternion(" << quaternion.getW() << ", " << quaternion.getX()
                    << ", " << quaternion.getY() << ", " << quaternion.getZ() << ")";
    return stream;
}

// Getters ========================================================================================

double quaternion::getW() const
{
    return w;
}

double quaternion::getX() const
{
    return x;
}

double quaternion::getY() const
{
    return y;
}

double quaternion::getZ() const
{
    return z;
}

double quaternion::getAngle() const
{
    return angle;
}

std::array<double, 3> quaternion::getAxis() const
{
    return axis;
}

/** \brief Compute the magnitude of the quaternion. */
double quaternion::norm() const
{
    static double n = std::sqrt(w*w + x*x + y*y + z*z);
    return n;
}

/** \brief Whether the quaternion has unit magnitude. */
bool quaternion::isNormal() const
{
    return norm() == 1;
}

/** \brief Create a new quaternion by normalizing this one. */
quaternion quaternion::normalized() const
{
    return *this / norm();
}

quaternion quaternion::unit() const
{
    return this->normalized();
}

/** \brief The conjugate is formed by negating all imaginary components. */
quaternion quaternion::conjugate() const
{
    return quaternion{w, -x, -y, -z};
}

/** \brief The inverse of a quaternion is its conjugate over the square of its magnitude. */
quaternion quaternion::inverse() const
{
    double n = norm();
    return conjugate()/(n*n);
}
