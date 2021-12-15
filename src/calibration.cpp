#include <iostream>
#include <tuple>
#include <vector>

#include "calibration.hpp"
#include "vector3.hpp"

void arwain::MagnetometerCalibrator::feed(const Vector3& reading)
{
    xyz = nc::stack({xyz, {reading.x, reading.y, reading.z}}, nc::Axis::ROW);
}

std::tuple<std::vector<double>, std::vector<std::vector<double>>> arwain::MagnetometerCalibrator::solve()
{
    // TODO Make sure there are enough data samples
    // TODO Make sure there is good sphere coverage
    // TODO Check for outliers?

    // Form A and b matrices.    
    nc::NdArray<double> xyz2 = xyz * xyz;
    nc::NdArray<double> xy = xyz(xyz.rSlice(), 0) * xyz(xyz.rSlice(), 1);
    nc::NdArray<double> xz = xyz(xyz.rSlice(), 0) * xyz(xyz.rSlice(), 2);
    nc::NdArray<double> yz = xyz(xyz.rSlice(), 1) * xyz(xyz.rSlice(), 2);
    nc::NdArray<double> A = nc::stack({xyz2, xy, xz, yz, xyz}, nc::Axis::COL);

    auto b = nc::ones<double>({A.shape().rows, 1});

    // Solve the system of lienar equations Ax = b for x.
    auto x = nc::linalg::lstsq(A, b);

    // Build the scaled ellipsoid quadric matrix in homogeneous coordinates.
    nc::NdArray<double> A2{
        {x(0, 0),     0.5*x(0, 3), 0.5*x(0, 4), 0.5*x(0, 6)},
        {0.5*x(0, 3),     x(0, 1), 0.5*x(0, 5), 0.5*x(0, 7)},
        {0.5*x(0, 4), 0.5*x(0, 5),     x(0, 2), 0.5*x(0, 8)},
        {0.5*x(0, 6), 0.5*x(0, 7), 0.5*x(0, 8),          -1}
    };

    // Build scaled ellipsoid quadric matrix in regular coordinates.
    nc::NdArray<double> Q{
        {x(0, 0),     0.5*x(0, 3), 0.5*x(0, 4)},
        {0.5*x(0, 3),     x(0, 1), 0.5*x(0, 5)},
        {0.5*x(0, 4), 0.5*x(0, 5),     x(0, 2)}
    };

    // Obtain the centroid of the ellispoid. This will be the bias offset vector.
    nc::NdArray<double> temp{0.5*x(0, 6), 0.5*x(0, 7), 0.5*x(0, 8)};
    temp.reshape(3, 1);
    nc::NdArray<double> x0 = nc::matmul(
        nc::linalg::inv(-1.0 * Q),
        temp
    );

    // Move the ellipsoid to the origin of the coordinate system.
    nc::NdArray<double> T_x0 = nc::eye<double>(4);
    T_x0(0, 3) = x0(0, 0);
    T_x0(1, 3) = x0(0, 1);
    T_x0(2, 3) = x0(0, 2);
    nc::NdArray<double> A3 = nc::matmul(nc::matmul(T_x0.transpose(), A2), T_x0);

    // Rescale the ellispsoid quadric matrix in regular coordinates.
    nc::NdArray<double> Q2 = Q * (-1.0 / A3(3, 3));

    // Take the Cholesky decomposition of Q. This matrix will transform the ellipsoid
    // onto a sphere, after correcting for bias offset.
    nc::NdArray<double> L = nc::eye<double>(3);
    try
    {
        L = nc::linalg::cholesky(Q2).transpose();
    }
    catch (std::runtime_error e)
    {
        L = nc::eye<double>(3);
    }

    return {
        {x0(0, 0), x0(0, 1), x0(0, 2)},
        {
            {L(0, 0), L(0, 1), L(0, 2)},
            {L(1, 0), L(1, 1), L(1, 2)},
            {L(2, 0), L(2, 1), L(2, 2)},
        }
    };
}
