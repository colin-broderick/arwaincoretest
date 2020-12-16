#include <math.h>
#include <array>

/// Computes the cross product of two 3-vectors. The result is a new 3-vector.
std::array<double, 3> cross(std::array<double, 3> v1, std::array<double, 3> v2);

/// Computes the L2 norm of a 3-vector as a double.
double norm(std::array<float, 3> arr);

/// Computes the L2 norm of a 3-vector as a double.
double norm(std::array<double, 3> arr);
