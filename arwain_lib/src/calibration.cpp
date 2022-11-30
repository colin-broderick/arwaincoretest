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

/** \brief Determine which region a specific 3-vector belongs in when projected onto the unit
 * sphere, where the sphere is split into 100 regions of equal area.
 *
 * Based on the work of Paul Stoffregen:
 *     https://github.com/PaulStoffregen/MotionCal

 * The sphere is divided into six bands. The northernmost band is a single region covering the
 * arctic cap. The southernmost band is also a single region, covering the antarctic cap. The
 * four remaining bands give two above and two below the equator. The bands adjacent the equator
 * are referred to as the tropic bands, and the bands adjacent the polar caps are referred to as
 * the temperate bands. Each band is split into regions such that the area of each is the same.
 * The northern tropic band is split into 15 regions along lines of longitude, each 24° apart.
 * The northern temperate band is split into 34 regions, each 10.58° apart. The two southern
 * bands are split into regions in the same way, giving a total of 100 regions.
 *
 * The following document describes the origin of the various numerical values in this function:
 *     https://etna.mcs.kent.edu/vol.25.2006/pp309-327.dir/pp309-327.html

 * The reading (x, y, z) is projected onto the sphere and this function returns an integer
 * corresponding to the region of the sphere in which it lands. The arctic cap is index 0; the
 * northern band regions are then numbered 1-49, and the southern bands 50-98, with the antarctic
 * cap being index 99.
 *
 * Within this function, all angles are represented in radian form.
 *
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

    const double arctic_cap_latitude = 1.37046;              // 78.52 degrees; the latitude of the line demarking the arctic cap.
    const double antarctic_cap_latitude = -1.37046;          // -78.52 degrees; the latitude of the line demarking the antarctic cap.
    const double north_tropic_temperate_boundary = 0.74776;  // 42.84 degrees; the latitude of the line separating the remaining two northern bands.
    const double south_tropic_temperate_boundary = -0.74776; // -42.84 degrees; the latitude of the line separating the remaining two southern bands.

    longitude = std::atan2(y, x) + pi;                             // Longitude in range [0, 2pi].
    latitude = pi / 2.0 - std::atan2(std::sqrt(x * x + y * y), z); // Latitude in range [-pi/2, +pi/2].

    if (latitude > arctic_cap_latitude) // This is the arctic cap, a singular region, index 0.
    {
        region = 0;
    }
    else if (latitude < antarctic_cap_latitude) // This is the antarctic cap, a singular region, index 99.
    {
        region = 99;
    }
    else if (latitude > north_tropic_temperate_boundary || latitude < south_tropic_temperate_boundary)
    {
        // We are in one of the temperate zones, 15 regions each.
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
            region += 1; // Northern temperate zones are indexed 1 to 15.
        }
        else
        {
            region += 84; // Southern temperate zones are indexed 84 to 98.
        }
    }
    else
    {
        // We are in one of the tropic zones, 34 regions each.
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
            region += 16; // Northern tropic zones are indexed 16 to 49.
        }
        else
        {
            region += 50; // Southern tropic zones are indexed 50 to 83.
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

    // TODO The 100 reads is to hopefully get enough data to ensure the initial bias parameters are
    // 'reasonable'. It should be possible to check that they are close enough and go from there, rather
    // than just hope that 100 reads is enough.
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

    // Form a 3x100 array, where each row of three elements is the average reading from the
    // corresponding sphere region.
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

    nc::NdArray<double> b = nc::ones<double>({A.shape().rows, 1});

    // Solve the system of lienar equations Ax = b for x.
    nc::NdArray<double> x = nc::linalg::lstsq(A, b);

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
    double average_delta = (delta.x + delta.y + delta.z) / 3.0;
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