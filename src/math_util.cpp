#include "math_util.h"
#include "quaternions.h"
#include "imu_utils.h"


/// Computes the L2 norm of a 3-vector as a double.
double norm(const std::array<double, 3>& arr)
{
    double res;
    res = arr[0] * arr[0] + arr[1] * arr[1] + arr[2] * arr[2];
    res = sqrt(res);
    return res;
}

/// Rotates a 3-vector according to a quaternion orientation.
std::array<double, 3> world_align(const std::array<double, 3>& vec, const quaternion& orientation)
{
    // Convert the 3-vector into a quaternion.
    quaternion quat_vec{0, vec[0], vec[1], vec[2]};
    quaternion quat_ori{orientation.w, orientation.x, orientation.y, orientation.z};

    // Compute the rotated vector as a quaternion.
    quaternion oriented_quat_vec = quat_ori * quat_vec * quat_ori.conj();

    // Cast the rotated quaternion back into a 3-vector.
    return std::array<double, 3>{
        (double)oriented_quat_vec.x,
        (double)oriented_quat_vec.y,
        (double)oriented_quat_vec.z
    };
}

/// Rotates a 3-vector according to a quaternion orientation.
vector3 world_align(const vector3& vec, const quaternion& orientation)
{
    // Convert the 3-vector into a quaternion.
    quaternion quat_vec{0, vec.x, vec.y, vec.z};
    quaternion quat_ori{orientation.w, orientation.x, orientation.y, orientation.z};

    // Compute the rotated vector as a quaternion.
    quaternion oriented_quat_vec = quat_ori * quat_vec * quat_ori.conj();

    // Cast the rotated quaternion back into a 3-vector.
    return vector3{
        (double)oriented_quat_vec.x,
        (double)oriented_quat_vec.y,
        (double)oriented_quat_vec.z
    };
}

/// Rotates a pair of 3-vectors, represented as a single 6-vector, according to a quaternion orientation.
std::array<double, 6> world_align(const std::array<double, 6>& imu, const quaternion& orientation)
{
    // Split out the acceleration and gyroscope parts of the 6-vector.
    std::array<double, 3> acce = std::array<double, 3>{
        imu[0], imu[1], imu[2]
    };
    std::array<double, 3> gyro = std::array<double, 3>{
        imu[3], imu[4], imu[5]
    };

    // World align the acceleration and gyroscope parts separately.
    std::array<double, 3> acce_aligned = world_align(acce, orientation);
    std::array<double, 3> gyro_aligned = world_align(gyro, orientation);

    // Recombine the accleration and gyro parts.
    return std::array<double, 6>{
        acce_aligned[0], acce_aligned[1], acce_aligned[2],
        gyro_aligned[0], gyro_aligned[1], gyro_aligned[2]
    };
}
