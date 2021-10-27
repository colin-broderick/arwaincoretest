#ifndef VECTOR3_H
#define VECTOR3_H

#include <math.h>
#include <iostream>

/** \brief Object for representing a 3-vector of various types, e.g. Eucliden displacement vector.
 */
struct vector3
{
    double x;
    double y;
    double z;
    double magnitude();
};

/** \brief Find the Euclidean (L2) norm of a 3-vector.
 * \return A double representing the length of the vector.
 */
inline double vector3::magnitude()
{
    return sqrt(x*x+y*y+z*z);
}

/** \brief The element-wise sum of two 3-vectors.
 * \param v1 A 3-vector.
 * \param v2 A second 3-vector to be added to the first.
 * \return A new 3-vector.
 */
inline vector3 operator+(const vector3 &v1, const vector3 &v2)
{
    return vector3{
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
inline vector3 operator-(const vector3 &v1, const vector3 &v2)
{
    return vector3{
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
template <class T> inline vector3 operator/(const vector3 &v, const T &scalar)
{
    return vector3{
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
inline vector3 operator*(const vector3 &v1, const vector3 &v2)
{
    return vector3{
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
template <class T> inline vector3 operator*(const vector3 &v, const T &scalar)
{
    return vector3{
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
template <class T> inline vector3 operator*(const T& scalar, const vector3& v)
{
    return v * scalar;
}

inline std::ostream& operator<<(std::ostream& stream, const vector3& vector)
{
    stream << "vector3(" << vector.x << ", " << vector.y << ", " << vector.z << ")";
    return stream;
}

inline vector3 cross(const vector3& v1, const vector3& v2)
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

inline vector3 normalised(vector3& vector)
{
    double invNorm = 1.0/vector.magnitude();
    return vector3{
        vector.x * invNorm,
        vector.y * invNorm,
        vector.z * invNorm
    };
}

struct vector6
{
    vector3 acce;
    vector3 gyro;
};

#endif
