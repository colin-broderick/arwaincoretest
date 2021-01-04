#include <math.h>
#include <array>
#include "utils.h"
#include "pi_utils.h"

/// Computes the cross product of two 3-vectors. The result is a new 3-vector.
std::array<double, 3> cross(std::array<double, 3> v1, std::array<double, 3> v2);

/// Computes the L2 norm of a 3-vector as a double.
double norm(std::array<float, 3> arr);

/// Computes the L2 norm of a 3-vector as a double.
double norm(std::array<double, 3> arr);

/// Rotates a 3-vector according to a quaternion orientation.
std::array<float, 3> world_align(std::array<float, 3> vec, quat_orientation_t orientation);

/// Rotates a 3-vector according to a quaternion orientation.
vec_scaled_output world_align(vec_scaled_output vec, quat_orientation_t orientation);

/// Rotates a pair of 3-vectors, represented as a single 6-vector, according to a quaternion orientation.
std::array<float, 6> world_align(std::array<float, 6> imu, quat_orientation_t orientation);