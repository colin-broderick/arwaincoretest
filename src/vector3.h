#ifndef VECTOR3_H
#define VECTOR3_H

#include <math.h>

/** \brief Object for representing a 3-vector of various types, e.g. Eucliden displacement vector.
 */
typedef struct vector3
{
    double x;
    double y;
    double z;
    double magnitude();
} vector3;

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
};

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
};

/** \brief Divide the elements of a 3-vector by a scalar value.
 * \param v A 3-vector.
 * \param scalar A scalar by which to divide the elements of the 3-vector.
 * \return A new 3-vector.
 */
template <class T>
inline vector3 operator/(const vector3 &v, const T &scalar)
{
    return vector3{
        v.x/scalar,
        v.y/scalar,
        v.z/scalar
    };
};

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
template <class T>
inline vector3 operator*(const vector3 &v, const T &scalar)
{
    return vector3{
        v.x*scalar,
        v.y*scalar,
        v.z*scalar
    };
};

#endif
