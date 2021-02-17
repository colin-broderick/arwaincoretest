#include "math_util.h"
#include "quaternions.h"
#include "imu_utils.h"


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

/// Rotates a 3-vector according to a quaternion orientation.
std::array<float, 3> world_align(std::array<float, 3> vec, quaternion orientation)
{
    // Convert the 3-vector into a quaternion.
    quaternion quat_vec((double)0, (double)vec[0], (double)vec[1], (double)vec[2]);
    quaternion quat_ori(orientation.w, orientation.x, orientation.y, orientation.z);

    // Compute the rotated vector as a quaternion.
    quaternion oriented_quat_vec = quat_ori * quat_vec * quat_ori.conj();

    // Cast the rotated quaternion back into a 3-vector.
    std::array<float, 3> oriented_vec = std::array<float, 3>{
        (float)oriented_quat_vec.x,
        (float)oriented_quat_vec.y,
        (float)oriented_quat_vec.z
    };

    return oriented_vec;
}

/// Rotates a 3-vector according to a quaternion orientation.
vector3 world_align(vector3 vec, quaternion orientation)
{
    // Convert the 3-vector into a quaternion.
    quaternion quat_vec((double)0, (double)vec.x, (double)vec.y, (double)vec.z);
    quaternion quat_ori(orientation.w, orientation.x, orientation.y, orientation.z);

    // Compute the rotated vector as a quaternion.
    quaternion oriented_quat_vec = quat_ori * quat_vec * quat_ori.conj();

    // Cast the rotated quaternion back into a 3-vector.
    vector3 oriented_vec{
        (float)oriented_quat_vec.x,
        (float)oriented_quat_vec.y,
        (float)oriented_quat_vec.z
    };

    return oriented_vec;
}

/// Rotates a pair of 3-vectors, represented as a single 6-vector, according to a quaternion orientation.
std::array<float, 6> world_align(std::array<float, 6> imu, quaternion orientation)
{
    // Split out the acceleration and gyroscope parts of the 6-vector.
    std::array<float, 3> acce = std::array<float, 3>{
        imu[0], imu[1], imu[2]
    };
    std::array<float, 3> gyro = std::array<float, 3>{
        imu[3], imu[4], imu[5]
    };

    // World align the acceleration and gyroscope parts separately.
    std::array<float, 3> acce_aligned = world_align(acce, orientation);
    std::array<float, 3> gyro_aligned = world_align(gyro, orientation);

    // Recombine the accleration and gyro parts.
    std::array<float, 6> imu_aligned = std::array<float, 6>{
        acce_aligned[0], acce_aligned[1], acce_aligned[2],
        gyro_aligned[0], gyro_aligned[1], gyro_aligned[2]
    };

    return imu_aligned;
}
