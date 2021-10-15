#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include <math.h>
#include <array>

#include "utils.hpp"
#include "quaternions.hpp"
#include "vector3.hpp"

/// Computes the cross product of two 3-vectors. The result is a new 3-vector.
std::array<double, 3> cross(const std::array<double, 3>& v1, const std::array<double, 3>& v2);
vector3 cross(const vector3& v1, const vector3& v2);

/// Computes the L2 norm of a 3-vector as a double.
double norm(const std::array<double, 3>& arr);

/// Rotates a 3-vector according to a quaternion orientation.
std::array<double, 3> world_align(const std::array<double, 3>& vec, const quaternion& orientation);

/// Rotates a 3-vector according to a quaternion orientation.
vector3 world_align(const vector3& vec, const quaternion& orientation);

/// Rotates a pair of 3-vectors, represented as a single 6-vector, according to a quaternion orientation.
std::array<double, 6> world_align(const std::array<double, 6>& imu, const quaternion& orientation);

template <class T> int sign(T value)
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
template <class T, class U> double smallest(T val1, U val2)
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
