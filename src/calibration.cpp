#include <iostream>
#include <tuple>
#include <vector>

#include "calibration.hpp"
#include "vector3.hpp"

static int sphere_coverage(int region_sample_count[100])
{
    int coverage = 0;
    for (int i = 0; i < 100; i++)
    {
        coverage += (region_sample_count[i] > 0);
    }
    return coverage;
}

/** \brief Having split the sphere into 100 regions of equal area, determine
 * which region a specific 3-vector belongs in when projected onto the unit
 * sphere. Based on the work of Paul Stoffregen:
 *      https://github.com/PaulStoffregen/MotionCal
 * \param x The x component of the 3-vector.
 * \param y The y component of the 3-vector.
 * \param z The z component of the 3-vector.
 * \return Integer corresponding to a sphere region label.
 */
static int sphere_region(double x, double y, double z)
{
    double latitude, longitude;
    int region;

    longitude = std::atan2(y, x) + M_PI;                             // longitude = 0 to 2pi  (meaning 0 to 360 degrees)
    latitude = M_PI / 2.0 - std::atan2(std::sqrt(x * x + y * y), z); // latitude = -pi/2 to +pi/2  (meaning -90 to +90 degrees)

    // https://etna.mcs.kent.edu/vol.25.2006/pp309-327.dir/pp309-327.html
    // sphere equations....
    //  area of unit sphere = 4*pi
    //  area of unit sphere cap = 2*pi*h  h = cap height
    //  lattitude of unit sphere cap = arcsin(1 - h)
    if (latitude > 1.37046f /* 78.52 deg */)
    {
        // arctic cap, 1 region
        region = 0;
    }
    else if (latitude < -1.37046f /* -78.52 deg */)
    {
        // antarctic cap, 1 region
        region = 99;
    }
    else if (latitude > 0.74776f /* 42.84 deg */ || latitude < -0.74776f)
    {
        // temperate zones, 15 regions each
        region = std::floor(longitude * 15.0 / (M_PI * 2.0));
        if (region < 0)
        {
            region = 0;
        }
        else if (region > 14)
        {
            region = 14;
        }
        if (latitude > 0.0)
        {
            region += 1; // 1 to 15
        }
        else
        {
            region += 84; // 84 to 98
        }
    }
    else
    {
        // tropic zones, 34 regions each
        region = std::floor(longitude * 34.0 / (M_PI * 2.0));
        if (region < 0)
        {
            region = 0;
        }
        else if (region > 33)
        {
            region = 33;
        }
        if (latitude >= 0.0)
        {
            region += 16; // 16 to 49
        }
        else
        {
            region += 50; // 50 to 83
        }
    }
    return region;
}

void arwain::MagnetometerCalibrator::feed(const Vector3& reading)
{
    static double x_bias = 0;
    static double y_bias = 0;
    static double z_bias = 0;
    static double x_min = 1000;
    static double x_max = -1000;
    static double y_min = 1000;
    static double y_max = -1000;
    static double z_min = 1000;
    static double z_max = -1000;

    // This first 100 samples are used to create a rough estimate of the biases, so that
    // sphere coverage can be accurately measured.
    if (feed_count < 100)
    {
        std::cout << "Determining bias parameters " << feed_count+1 << std::endl;
        if (reading.x < x_min) x_min = reading.x;
        if (reading.x > x_max) x_max = reading.x;
        if (reading.y < y_min) y_min = reading.y;
        if (reading.y > y_max) y_max = reading.y;
        if (reading.z < z_min) z_min = reading.z;
        if (reading.z > z_max) z_max = reading.z;
        x_bias = (x_min + x_max) / 2.0;
        y_bias = (y_min + y_max) / 2.0;
        z_bias = (z_min + z_max) / 2.0;
        feed_count++;
        return;
    }

    if (feed_count == 100)
    {
        std::cout << std::endl;
        feed_count++;
    }

    // Once the biases are approximately established, record each data sample into the
    // appropriate sphere region.
    int region = sphere_region(reading.x - x_bias, reading.y - y_bias, reading.z - z_bias);
    region_sample_count[region]++;
    region_sample_value[region] = region_sample_value[region] + reading;

    this->sphere_coverage_quality = sphere_coverage(region_sample_count);

    std::cout << "Coverage quality: " << this->sphere_coverage_quality << std::endl;
}

int arwain::MagnetometerCalibrator::get_sphere_coverage_quality()
{
    return this->sphere_coverage_quality;
}

std::tuple<std::vector<double>, std::vector<std::vector<double>>> arwain::MagnetometerCalibrator::solve()
{
    // TODO Make sure there are enough data samples
    // TODO Make sure there is good sphere coverage
    // TODO Check for outliers?

    for (int i = 0; i < 100; i++)
    {
        if (region_sample_count[i] != 0)
        {
            xyz = nc::stack({xyz, {
                region_sample_value[i].x / (double)(region_sample_count[i]),
                region_sample_value[i].y / (double)(region_sample_count[i]),
                region_sample_value[i].z / (double)(region_sample_count[i])
            }}, nc::Axis::ROW);
        }
    }

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
        {x(0, 0), 0.5 * x(0, 3), 0.5 * x(0, 4), 0.5 * x(0, 6)},
        {0.5 * x(0, 3), x(0, 1), 0.5 * x(0, 5), 0.5 * x(0, 7)},
        {0.5 * x(0, 4), 0.5 * x(0, 5), x(0, 2), 0.5 * x(0, 8)},
        {0.5 * x(0, 6), 0.5 * x(0, 7), 0.5 * x(0, 8), -1}};

    // Build scaled ellipsoid quadric matrix in regular coordinates.
    nc::NdArray<double> Q{
        {x(0, 0), 0.5 * x(0, 3), 0.5 * x(0, 4)},
        {0.5 * x(0, 3), x(0, 1), 0.5 * x(0, 5)},
        {0.5 * x(0, 4), 0.5 * x(0, 5), x(0, 2)}};

    // Obtain the centroid of the ellispoid. This will be the bias offset vector.
    nc::NdArray<double> temp{0.5 * x(0, 6), 0.5 * x(0, 7), 0.5 * x(0, 8)};
    temp.reshape(3, 1);
    nc::NdArray<double> x0 = nc::matmul(
        nc::linalg::inv(-1.0 * Q),
        temp);

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
    catch (const std::runtime_error& e)
    {
        L = nc::eye<double>(3);
    }

    return {
        {
            x0(0, 0), x0(0, 1), x0(0, 2)
        },
        {
            {L(0, 0), L(0, 1), L(0, 2)},
            {L(1, 0), L(1, 1), L(1, 2)},
            {L(2, 0), L(2, 1), L(2, 2)},
        }
    };
}
