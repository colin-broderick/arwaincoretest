#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include <math.h>
#include <array>
#include "utils.h"
#include "imu_utils.h"
#include "quaternions.h"

/// Computes the cross product of two 3-vectors. The result is a new 3-vector.
template <class T>
T cross(T v1, T v2)
{
    /*
    Computes the cross product using determinant formula:

    a x b = |  i   j   k | = i(a1a1 - b1a2) + j(a0b2 - b0a2) + k(a0b1 - b0a1)
            | a0  a1  a2 |
            | b0  b1  b2 |

    */
    T res;
    res.x = v1.y * v2.z - v2.y * v1.z;
    res.y = -(v1.x * v2.z - v2.x * v1.z);
    res.z = v1.x * v2.y - v2.x * v1.y;
    return res;
}

/// Computes the L2 norm of a 3-vector as a double.
double norm(std::array<double, 3> arr);

/// Computes the L2 norm of a 3-vector as a double.
double norm(std::array<double, 3> arr);

/// Rotates a 3-vector according to a quaternion orientation.
std::array<double, 3> world_align(std::array<double, 3> vec, quaternion orientation);

/// Rotates a 3-vector according to a quaternion orientation.
vector3 world_align(vector3 vec, quaternion orientation);

/// Rotates a pair of 3-vectors, represented as a single 6-vector, according to a quaternion orientation.
std::array<double, 6> world_align(std::array<double, 6> imu, quaternion orientation);

template <class T>
int sign(T value)
{
    if (value < 0)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}

// Determines which of a pair of numbers is closer to zero, i.e. smallest magnitude.
template <class T, class U>
double smallest(T val1, U val2)
{
    if (abs(val1) < abs(val2))
    {
        return (double)val1;
    }
    else
    {
        return (double)val2;
    }
}

#endif
