#include <iostream>
#include <tuple>
#include <vector>

#include "calibration.hpp"
#include "vector3.hpp"

/** \brief Computes the coverage percentage of a sphere by counting the number of non-zero
 * entries in the coverage array. Each index in the array corresponds to an equal-area region
 * of a sphere.
 * \param region_sample_count Coverage array; each non-zero element corresponds to a sphere region
 * for which data has been acquired. Each zero element corresponds to a sphere region for which
 * data has not been acquired.
 */
int MagnetometerCalibrator::sphere_coverage(const std::array<int, 100>& region_sample_count) const
{
    int coverage = 0;
    for (const int value : region_sample_count)
    {
        coverage += (value > 0);
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
int MagnetometerCalibrator::sphere_region(const double x, const double y, const double z) const
{
    double latitude = 0;
    double longitude = 0;
    int region = 0;

    longitude = std::atan2(y, x) + pi;                             // longitude = 0 to 2pi  (meaning 0 to 360 degrees)
    latitude = pi / 2.0 - std::atan2(std::sqrt(x * x + y * y), z); // latitude = -pi/2 to +pi/2  (meaning -90 to +90 degrees)

    // https://etna.mcs.kent.edu/vol.25.2006/pp309-327.dir/pp309-327.html
    // sphere equations....
    //  area of unit sphere = 4*pi
    //  area of unit sphere cap = 2*pi*h  h = cap height
    //  lattitude of unit sphere cap = arcsin(1 - h)
    if (latitude > 1.37046 /* 78.52 deg */)
    {
        // arctic cap, 1 region
        region = 0;
    }
    else if (latitude < -1.37046 /* -78.52 deg */)
    {
        // antarctic cap, 1 region
        region = 99;
    }
    else if (latitude > 0.74776 /* 42.84 deg */ || latitude < -0.74776)
    {
        // temperate zones, 15 regions each
        region = std::floor(longitude * 15.0 / (pi * 2.0));
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
        region = std::floor(longitude * 34.0 / (pi * 2.0));
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

/** \brief Continually feed magnetometer readings into this function while randomly tumbling
 * the device around all axes. This function assumes that the first 100 readings will give 
 * sufficient sphere coverage to provide a reasonable estimate of bias parameters.
 * For that reason, there should be a short delay between each reading.
 * 
 * By 'reasonable' estimate of bias parameters, we here mean that, after calculation and
 * application of those bias parameters, the origin of the coordinate system is contained
 * within the sphere surface. If the bias parameters are not sufficiently accurate for this
 * purpose, later stages of the calibration will likely fail. After the first 100 readings,
 * subsequent readings will be used to accumulate sphere coverage with respect to the
 * newly-acquired bias parameters.
 */
void MagnetometerCalibrator::feed(const Vector3& reading)
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
        std::cout << "Determining bias parameters " << feed_count+1 << "\n";
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
        std::cout << "\n";
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

/** \brief Gives a measure of fully the sphere has been covered by data readings fed to the
 * calibrator. Quality is given as a percentage, where 100% means all 100 regions of the sphere
 * have at least one associated measurement.
 * \return Sphere coverage quality as a percentage. 
 */
int MagnetometerCalibrator::get_sphere_coverage_quality() const
{
    return this->sphere_coverage_quality;
}

/** \brief Generates magnetometer calibration parameters if enough data has been collected.
 * \return A tuple of vectors, where:
 * the first vector is the bias parameters in the order x, y, z;
 * the second vector is the scale parameters in the arrangement
 *    {x      xy cross    xz cross},
 *    {.      y           yz cross},
 *    {.      .           z       },
 * The blank entries can be ignored.
 */
std::tuple<std::vector<double>, std::vector<std::vector<double>>> MagnetometerCalibrator::solve()
{
    // TODO Make sure there are enough data samples
    // TODO Make sure there is good sphere coverage
    // TODO Check for outliers?

    for (int i = 0; i < 100; i++)
    {
        if (region_sample_count[i] != 0)
        {
            xyz = nc::stack({xyz, {
                region_sample_value[i].x / static_cast<double>(region_sample_count[i]),
                region_sample_value[i].y / static_cast<double>(region_sample_count[i]),
                region_sample_value[i].z / static_cast<double>(region_sample_count[i])
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
            x0(0, 0), x0(0, 1), x0(0, 2)  // bias parameters.
        },
        {
            {L(0, 0), L(0, 1), L(0, 2)},  // scale parameters.
            {L(1, 0), L(1, 1), L(1, 2)},
            {L(2, 0), L(2, 1), L(2, 2)},
        }
    };
}

/** \brief Reports whether the current series of readings has converged.
 * 
 * Here, converged means that the state estimater has reached a certain confidence level for the
 * readings being fed in, where it as assumed that the readings are all taken with the device
 * stationary in some fixed orientation.
 *
 * \return Boolean indicated whether the state estimater has converged.
 */
bool AccelerometerCalibrator::is_converged()
{
    return converged;
}

/** \brief Get the currently estimated state for all three accelerometer axes. 
 * \return Current state estimate.
 */
Vector3 AccelerometerCalibrator::get_params()
{
    return {kfx.est, kfy.est, kfz.est};
}

/** \brief Call to tell the calibrator that measurement of accelerometer data in the current orientation
 * has been finished, and therefore the internal state estimater and convergence state should be reset.
 *
 * After calling next_sampling(), start feeding data for a new orientation or continue
 * on to deduce_calib_params().
 */
void AccelerometerCalibrator::next_sampling()
{
    converged = false;
    samplings.push_back({kfx.est, kfy.est, kfz.est});
    kfx = KalmanFilter1D{9.81, 1.0};
    kfy = KalmanFilter1D{9.81, 1.0};
    kfz = KalmanFilter1D{9.81, 1.0};
}

/** \brief Supply a new data reading.
 * \param reading Accelerometer data in units of m/s2.
 * \return Boolean indicating whether convergence has been reached,
 */
bool AccelerometerCalibrator::feed(const Vector3& reading)
{
    kfx.update(reading.x, 0.1);
    kfy.update(reading.y, 0.1);
    kfz.update(reading.z, 0.1);

    if (!kfx.converged || !kfy.converged || !kfz.converged)
    {
        converged = false;
    }
    else
    {
        converged = true;
    }

    return converged;
}

/** \brief Using data collected so far, calculate bias and scale parameters to calibrate the accelerometers.
 * \return A pair of vectors, where the first vector is the offset parameters in the order x, y, z, and the
 * second vector is scale parameters in the order x, y, z.
 */
std::tuple<Vector3, Vector3> AccelerometerCalibrator::deduce_calib_params()
{
    double x_min = 1e6;
    double x_max = -1e6;
    double y_min = 1e6;
    double y_max = -1e6;
    double z_min = 1e6;
    double z_max = -1e6;
    
    for (const Vector3& vec : samplings)
    {
        x_min = vec.x < x_min ? vec.x : x_min;
        x_max = vec.x > x_max ? vec.x : x_max;
        y_min = vec.y < y_min ? vec.y : y_min;
        y_max = vec.y > y_max ? vec.y : y_max;
        z_min = vec.z < z_min ? vec.z : z_min;
        z_max = vec.z > z_max ? vec.z : z_max;
    }
    Vector3 bias_ = {(x_min + x_max) / 2.0, (y_min + y_max) / 2.0, (z_min + z_max) / 2.0};

    // Compute scale correction factors.
    Vector3 delta = {(x_max - x_min) / 2.0, (y_max - y_min) / 2.0, (z_max - z_min) / 2.0};
    double average_delta = (delta.x + delta.y + delta.z)/3.0;
    double scale_x = average_delta / delta.x;
    double scale_y = average_delta / delta.y;
    double scale_z = average_delta / delta.z;
    Vector3 scale_ = {scale_x, scale_y, scale_z};

    return {bias_, scale_};
}

/** \brief Reports whether the current series of readings has converged.
 * 
 * Here, converged means that the state estimater has reached a certain confidence level for the
 * readings being fed in, where it as assumed that the readings are all taken with the device
 * stationary in some fixed orientation.
 *
 * \return Boolean indicated whether the state estimater has converged.
 */
bool GyroscopeCalibrator::is_converged()
{
    return converged;
}

/** \brief Get the currently estimated state for all three gyroscope axes. 
 * \return Current state estimate.
 */
Vector3 GyroscopeCalibrator::get_params()
{
    return {kfx.est, kfy.est, kfz.est};
}

/** \brief Supply a new data reading.
 * \param reading Gyroscope data in units of rad/s.
 * \return Boolean indicating whether convergence has been reached,
 */
bool GyroscopeCalibrator::feed(const Vector3& reading)
{
    kfx.update(reading.x, 0.02);
    kfy.update(reading.y, 0.02);
    kfz.update(reading.z, 0.02);

    if (!kfx.converged || !kfy.converged || !kfz.converged)
    {
        converged = false;
    }
    else
    {
        converged = true;
    }

    return converged;
}