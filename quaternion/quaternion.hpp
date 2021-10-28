#ifndef _ARWAIN_QUATERNIONS_HPP
#define _ARWAIN_QUATERNIONS_HPP

#include <iostream>
#include <cmath>
#include <array>

// Class layout ===================================================================================

class quaternion
{
    private:
        double angle;
        std::array<double, 3> axis;

    public:
        double w, x, y, z;

        // Constructors
        quaternion();
        quaternion(const std::array<double, 3>& vec);
        quaternion(const double angle, const std::array<double, 3>& axis);
        quaternion(const double w, const double x, const double y, const double z);

        // Static methods
        static double dotProduct(const quaternion& quat1, const quaternion& quat2);
        static quaternion slerp(quaternion quat1, const quaternion& quat2, const double t);
        static quaternion nslerp(quaternion quat1, const quaternion& quat2, const double t);

        // Getters
        double getW() const;
        double getX() const;
        double getY() const;
        double getZ() const;
        double getAngle() const;
        std::array<double, 3> getAxis() const;
        double norm() const;
        bool isNormal() const;
        quaternion unit() const;
        quaternion normalized() const;
        quaternion conjugate() const;
        quaternion inverse() const;
};

bool operator==(const quaternion& quat1, const quaternion& quat2);
quaternion operator+(const quaternion& quat1, const quaternion& quat2);
quaternion operator-(const quaternion& quat1, const quaternion& quat2);
quaternion operator*(const quaternion& quat1, const quaternion& quat2);
quaternion operator-(const quaternion& quat);
bool operator==(const quaternion& quat1, const quaternion& quat2);
std::ostream& operator<<(std::ostream& stream, const quaternion& quat);

/** \brief Multiply a quaternion by a scalar value.
 * \param scalar The scalar value by which to multiply.
 * \param quat The quaternion to multiply.
 * \return The new quaternion after multiplication.
 */
template<typename Scalar>
quaternion operator*(const Scalar scalar, const quaternion& quat)
{
    return quaternion{
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
quaternion operator*(const quaternion& quat, const Scalar scalar)
{
    return scalar * quat;
}

/** \brief Divide a quaternion by a scalar value, i.e. multiply that that values inverse.
 * \param quaternion The quaternion to divide.
 * \param divisor The scalar value by which to divide.
 * \return The new quaternion after division.
 */
template<typename Divisor>
quaternion operator/(const quaternion& quat, const Divisor divisor)
{
    return (1.0/divisor) * quat;
}

#endif
