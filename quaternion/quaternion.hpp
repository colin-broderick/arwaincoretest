#ifndef _ARWAIN_QUATERNIONS_HPP
#define _ARWAIN_QUATERNIONS_HPP

#include <iostream>
#include <cmath>
#include <array>

#include "vector3.hpp"

// Class layout ===================================================================================

class Quaternion
{
    private:
        double angle;
        std::array<double, 3> axis;

    public:
        double w, x, y, z;

        // Constructors
        Quaternion();
        Quaternion(const std::array<double, 3>& vec);
        Quaternion(const double angle, const std::array<double, 3>& axis);
        Quaternion(const double w, const double x, const double y, const double z);

        // Static methods
        static double dot(const Quaternion& quat1, const Quaternion& quat2);
        static Quaternion slerp(Quaternion quat1, const Quaternion& quat2, const double t);
        static Quaternion nslerp(Quaternion quat1, const Quaternion& quat2, const double t);
        static double angle_between(const Quaternion& q1, const Quaternion& q2);

        // Getters
        double getW() const;
        double getX() const;
        double getY() const;
        double getZ() const;
        double getAngle() const;
        Vector3 vector_part() const;
        std::array<double, 3> getAxis() const;
        double norm() const;
        bool isNormal() const;
        Quaternion unit() const;
        Quaternion normalized() const;
        Quaternion conjugate() const;
        Quaternion inverse() const;
};

bool operator==(const Quaternion& quat1, const Quaternion& quat2);
Quaternion operator+(const Quaternion& quat1, const Quaternion& quat2);
Quaternion operator-(const Quaternion& quat1, const Quaternion& quat2);
Quaternion operator*(const Quaternion& quat1, const Quaternion& quat2);
Quaternion operator-(const Quaternion& quat);
bool operator==(const Quaternion& quat1, const Quaternion& quat2);
std::ostream& operator<<(std::ostream& stream, const Quaternion& quat);

/** \brief Multiply a quaternion by a scalar value.
 * \param scalar The scalar value by which to multiply.
 * \param quat The quaternion to multiply.
 * \return The new quaternion after multiplication.
 */
template<typename Scalar>
Quaternion operator*(const Scalar scalar, const Quaternion& quat)
{
    return Quaternion{
        scalar * quat.getW(),
        scalar * quat.getX(),
        scalar * quat.getY(),
        scalar * quat.getZ()
    };
}

/** \brief Multiply a quaternion by a scalar value.
 * \param quaternion The quaternion to multiply.
 * \param scalar The scalar value by which to multiply.
 * \return The new quaternion after multiplication.
 */
template<typename Scalar>
Quaternion operator*(const Quaternion& quat, const Scalar scalar)
{
    return scalar * quat;
}

/** \brief Divide a quaternion by a scalar value, i.e. multiply that that values inverse.
 * \param quaternion The quaternion to divide.
 * \param divisor The scalar value by which to divide.
 * \return The new quaternion after division.
 */
template<typename Divisor>
Quaternion operator/(const Quaternion& quat, const Divisor divisor)
{
    return (1.0/divisor) * quat;
}

#endif
