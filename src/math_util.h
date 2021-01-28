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
    res[0] = v1[1] * v2[2] - v2[1] * v1[2];
    res[1] = v1[0] * v2[2] - v2[0] * v1[2];
    res[2] = v1[0] * v2[1] - v2[0] * v1[1];
    return res;
}

/// Computes the L2 norm of a 3-vector as a double.
double norm(std::array<float, 3> arr);

/// Computes the L2 norm of a 3-vector as a double.
double norm(std::array<double, 3> arr);

/// Rotates a 3-vector according to a quaternion orientation.
std::array<float, 3> world_align(std::array<float, 3> vec, quaternion orientation);

/// Rotates a 3-vector according to a quaternion orientation.
vec_scaled_output world_align(vec_scaled_output vec, quaternion orientation);

/// Rotates a pair of 3-vectors, represented as a single 6-vector, according to a quaternion orientation.
std::array<float, 6> world_align(std::array<float, 6> imu, quaternion orientation);

#endif
