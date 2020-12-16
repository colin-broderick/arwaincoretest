#include "math_util.h"


/// Computes the cross product of two 3-vectors. The result is a new 3-vector.
std::array<double, 3> cross(std::array<double, 3> v1, std::array<double, 3> v2)
{
    std::array<double, 3> res;
    res[0] = v1[1] * v2[2] - v2[1] * v1[2];
    res[1] = v1[0] * v2[2] - v2[0] * v1[2];
    res[2] = v1[0] * v2[1] - v2[0] * v1[1];
    return res;
}

/// Computes the L2 norm of a 3-vector as a double.
double norm(std::array<float, 3> arr)
{
    double res;
    res = arr[0] * arr[0] + arr[1] * arr[1] + arr[2] * arr[2];
    res = sqrt(res);
    return (double)res;
}

/// Computes the L2 norm of a 3-vector as a double.
double norm(std::array<double, 3> arr)
{
    double res;
    res = arr[0] * arr[0] + arr[1] * arr[1] + arr[2] * arr[2];
    res = sqrt(res);
    return res;
}
