#ifndef Vector3_H
#define Vector3_H

#include <cmath>
#include <iostream>

/** \brief Object for representing a 3-vector of various types, e.g. Eucliden displacement vector.
 */
struct Vector3
{
    double x;
    double y;
    double z;
    double magnitude() const;
    Vector3 normalized();
    static Vector3 cross(const Vector3& v1, const Vector3& v2);
    static double angle_between(const Vector3& v1, const Vector3& v2);
    static double dot(const Vector3& v1, const Vector3& v2);
};

/** \brief Find the Euclidean (L2) norm of a 3-vector.
 * \return A double representing the length of the vector.
 */
inline double Vector3::magnitude() const
{
    return sqrt(x*x+y*y+z*z);
}

/** \brief The element-wise sum of two 3-vectors.
 * \param v1 A 3-vector.
 * \param v2 A second 3-vector to be added to the first.
 * \return A new 3-vector.
 */
inline Vector3 operator+(const Vector3 &v1, const Vector3 &v2)
{
    return Vector3{
        v1.x + v2.x,
        v1.y + v2.y,
        v1.z + v2.z
    };
}

/** \brief The element-wise difference of two 3-vectors.
 * \param v1 A 3-vector.
 * \param v2 A second 3-vector to be subtracted from the first.
 * \return A new 3-vector.
 */
inline Vector3 operator-(const Vector3 &v1, const Vector3 &v2)
{
    return Vector3{
        v1.x - v2.x,
        v1.y - v2.y,
        v1.z - v2.z
    };
}

/** \brief Divide the elements of a 3-vector by a scalar value.
 * \param v A 3-vector.
 * \param scalar A scalar by which to divide the elements of the 3-vector.
 * \return A new 3-vector.
 */
template <class T> inline Vector3 operator/(const Vector3 &v, const T &scalar)
{
    return Vector3{
        v.x/scalar,
        v.y/scalar,
        v.z/scalar
    };
}

/** \brief Element-wise product of two 3-vectors.
 * \param v1 A 3-vector.
 * \param v2 A second 3-vector.
 * \return A new 3-vector.
 */
inline Vector3 operator*(const Vector3 &v1, const Vector3 &v2)
{
    return Vector3{
        v1.x*v2.x,
        v1.y*v2.y,
        v1.z*v2.z
    };
}

/** \brief Multiply the elements of a 3-vector by a scalar value.
 * \param v A 3-vector.
 * \param scalar A scalar by which to multiply the elements of the 3-vector.
 * \return A new 3-vector.
 */
template <class T> inline Vector3 operator*(const Vector3 &v, const T &scalar)
{
    return Vector3{
        v.x*scalar,
        v.y*scalar,
        v.z*scalar
    };
}

/** \brief Multiply the elements of a 3-vector by a scalar value.
 * \param scalar A scalar by which to multiply the elements of the 3-vector.
 * \param v A 3-vector.
 * \return A new 3-vector.
 */
template <class T> inline Vector3 operator*(const T& scalar, const Vector3& v)
{
    return v * scalar;
}

inline std::ostream& operator<<(std::ostream& stream, const Vector3& vector)
{
    stream << "Vector3(" << vector.x << ", " << vector.y << ", " << vector.z << ")";
    return stream;
}

inline Vector3 Vector3::cross(const Vector3& v1, const Vector3& v2)
{
    /*
    Computes the cross product using determinant formula:

    a x b = |  i   j   k | = i(a1a1 - b1a2) + j(a0b2 - b0a2) + k(a0b1 - b0a1)
            | a0  a1  a2 |
            | b0  b1  b2 |

    */
    return {
        v1.y * v2.z - v2.y * v1.z,
        -(v1.x * v2.z - v2.x * v1.z),
        v1.x * v2.y - v2.x * v1.y
    };
}

inline Vector3 Vector3::normalized()
{
    double invNorm = 1.0/this->magnitude();
    return Vector3{
        this->x * invNorm,
        this->y * invNorm,
        this->z * invNorm
    };
}

inline double Vector3::dot(const Vector3& v1, const Vector3& v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.y;
}

inline double Vector3::angle_between(const Vector3& v1, const Vector3& v2)
{
    return std::acos(Vector3::dot(v1, v2) / v1.magnitude() / v2.magnitude() );
}

struct Vector6
{
    Vector3 acce;
    Vector3 gyro;
};

#endif
