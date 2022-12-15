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
int MagnetometerCalibrator::sphere_coverage(const std::array<int, MagnetometerCalibrator::total_sphere_regions>& region_sample_count) const
{
    int coverage = 0;
    for (const int value : region_sample_count)
    {
        coverage += (value > 0);
    }
    return coverage;
}

/** \brief Get count of the number of data readings that have been supplied to the calibrator.
 * The feed count is incremented each time feed() is called. 
 * \return Integer count of readings supplied.
 */
int MagnetometerCalibrator::get_feed_count() const
{
    return feed_count;
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
    latitude = pi / 2.0 - std::atan2(std::sqrt((x * x) + (y * y)), z); // Latitude in range [-pi/2, +pi/2].

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
    // 1000 is just a number sufficiently large that now real reading will fall outside these bounds.
    static double x_min = 1000;
    static double x_max = -1000;
    static double y_min = 1000;
    static double y_max = -1000;
    static double z_min = 1000;
    static double z_max = -1000;
    
    static double x_bias = 0;
    static double y_bias = 0;
    static double z_bias = 0;

    // This first 100 samples are used to create a rough estimate of the biases, so that
    // sphere coverage can be accurately measured.
    if (feed_count < 100)
    {
        std::cout << "Determining bias parameters " << feed_count + 1 << "\n";
        if (reading.x < x_min) x_min = reading.x;
        if (reading.x > x_max) x_max = reading.x;
        if (reading.y < y_min) y_min = reading.y;
        if (reading.y > y_max) y_max = reading.y;
        if (reading.z < z_min) z_min = reading.z;
        if (reading.z > z_max) z_max = reading.z;
        feed_count++;
        return;
    }

    // TODO The 100 reads is to hopefully get enough data to ensure the initial bias parameters are
    // 'reasonable'. It should be possible to check that they are close enough and go from there, rather
    // than just hope that 100 reads is enough.
    if (feed_count == 100)
    {
        x_bias = (x_min + x_max) / 2.0;
        y_bias = (y_min + y_max) / 2.0;
        z_bias = (z_min + z_max) / 2.0;
        std::cout << "\n";
    }
    
    feed_count++;

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

/** \brief Gets the current region sample count.
 * \return An array where each index contains an int which counts the number of readings
 * which correspond to the sphere region with that index.
 */
std::array<int, MagnetometerCalibrator::total_sphere_regions> MagnetometerCalibrator::get_region_sample_count() const
{
    return region_sample_count;
}

/** \brief Gets the current cumulative readings for each sphere region.
 * \return An array where each element is a Vector3 giving the sum of all readings fed in for
 * the corresponding sphere region.
 */
std::array<Vector3, MagnetometerCalibrator::total_sphere_regions> MagnetometerCalibrator::get_region_sample_value() const
{
    return region_sample_value;
}

/** \brief Forms a data matrix where the elements are ordered to agree with the standard ordering
 * of a quadric parameter matrix. If the input matrix has rows of form [x, y, z], the returned matrix
 * has form
 *     [x^2, y^2, z^2, xy, yz, xz, x, y, z]
 * \param data_array An array with 3 columns and any number of rows.
 * \return A horizontally stacked array as described in the brief description.
 */
nc::NdArray<double> MagnetometerCalibrator::form_augmented_data_array(const nc::NdArray<double>& data_array)
{
    // data_array_2 is the element-wise product of the data_array
    nc::NdArray<double> data_array_2 = data_array * data_array;

    // The xy, xy, and yz are the (num_readings, 1) matrix form by taking the element-wise product of, e.g.,
    // the x column and the y column of the data array.
    nc::NdArray<double> xy = data_array(data_array.rSlice(), 0) * data_array(data_array.rSlice(), 1);
    nc::NdArray<double> xz = data_array(data_array.rSlice(), 0) * data_array(data_array.rSlice(), 2);
    nc::NdArray<double> yz = data_array(data_array.rSlice(), 1) * data_array(data_array.rSlice(), 2);

    // We stack the above to form a matrix with (num_readings) rows and nine columns.
    return nc::stack({data_array_2, xy, xz, yz, data_array}, nc::Axis::COL);
}

/** \brief Form a (total_sphere_regions, 3) array, where each row of three elements is the average reading from the
 * corresponding sphere region. If any of the sphere regions do not contain any data, they will
 * be ignored and the array may actually have fewer than total_sphere_regions rows.
 * \param region_sample_counts A 100-long array of ints, where each int is the count of data sample from that sphere region.
 * \param region_sample_values A 100-long array of Vector3s, where each Vector3 is the cumulative value of readings in the
 * corresponding sphere region.
 * \return A 100x3 array, where each row of 3 represents the average value of the reading in some particular sphere region.
 */
nc::NdArray<double> MagnetometerCalibrator::form_data_array(const std::array<int, total_sphere_regions>& region_sample_counts, const std::array<Vector3, total_sphere_regions>& region_sample_values)
{
    nc::NdArray<double> data_array;
    for (int i = 0; i < total_sphere_regions; i++)
    {
        if (region_sample_counts[i] != 0)
        {
            data_array = nc::stack({data_array, {
                region_sample_values[i].x / static_cast<double>(region_sample_counts[i]),
                region_sample_values[i].y / static_cast<double>(region_sample_counts[i]),
                region_sample_values[i].z / static_cast<double>(region_sample_counts[i])
            }}, nc::Axis::ROW);
        }
    }
    return data_array;
}

/** \brief Creates the matrix describing an ellipsoid in homogeneous coordinates.
 * It is assumed that the constant term is -1, so only 9 elements coefficients need to be passed, 
 * and the bottom-right element of the generated matrix will be equal to -1.
 * \param coefficients A [1,9] matrix where the elements are the cofficients of an ellipsoid polynomial in stanard form.
 * \return A [4,4] matrix representing the ellipsoid.
*/
nc::NdArray<double> MagnetometerCalibrator::create_ellipsoid_homogeneous(const nc::NdArray<double>& coefficients)
{
    return {
        {      coefficients(0, 0), 0.5 * coefficients(0, 3), 0.5 * coefficients(0, 4), 0.5 * coefficients(0, 6)},
        {0.5 * coefficients(0, 3),       coefficients(0, 1), 0.5 * coefficients(0, 5), 0.5 * coefficients(0, 7)},
        {0.5 * coefficients(0, 4), 0.5 * coefficients(0, 5),       coefficients(0, 2), 0.5 * coefficients(0, 8)},
        {0.5 * coefficients(0, 6), 0.5 * coefficients(0, 7), 0.5 * coefficients(0, 8),                 -1}
    };
}

/** \brief Forms a matrix representing the square terms of an ellipsoid in regular form.
 * \param coefficients The coefficient matrix of the ellipse.
 * \return 3x3 matrix representing square terms of ellipsoid.
 */
nc::NdArray<double> MagnetometerCalibrator::create_ellipsoid_regular(const nc::NdArray<double>& coefficients)
{
    return {
        {      coefficients(0, 0), 0.5 * coefficients(0, 3), 0.5 * coefficients(0, 4)},
        {0.5 * coefficients(0, 3),       coefficients(0, 1), 0.5 * coefficients(0, 5)},
        {0.5 * coefficients(0, 4), 0.5 * coefficients(0, 5),       coefficients(0, 2)}
    };
}

/** \brief Creates an operator which translates a quadric in homogeneous form. 
 * \param vector The vector by which to translate the quadric.
 * \return The translation operator.
 */
nc::NdArray<double> MagnetometerCalibrator::create_translation_operator_homogeneous(const nc::NdArray<double>& vector)
{
    nc::NdArray<double> offset_operator = nc::eye<double>(4);
    offset_operator(0, 3) = vector(0, 0);
    offset_operator(1, 3) = vector(0, 1);
    offset_operator(2, 3) = vector(0, 2);
    return offset_operator;
}

/** \brief Generates magnetometer calibration parameters if enough data has been collected.
 *
 * The procedure is described in detail in the software design document U0021026.
 * 
 * \return A tuple of vectors, where:
 * the first vector is the bias parameters in the order x, y, z;
 * the second vector is the scale parameters in the arrangement
 *    {x      xy cross    xz cross},
 *    {.      y           yz cross},
 *    {.      .           z       },
 * The blank entries can be ignored.
 */
MagnetometerCalibrator::CalibrationParameters MagnetometerCalibrator::solve()
{
    // TODO Make sure there are enough data samples
    // TODO Make sure there is good sphere coverage
    // TODO Check for outliers?
    // TODO Raise some kind of error if Cholesky decomposition fails, instead of quietly returning
    // 'invalid' result.

    // Form the array of 3-vectors which represent magnetometer readings.
    nc::NdArray<double> data_array = form_data_array(region_sample_count, region_sample_value);

    // Form A and b matrices.
    nc::NdArray<double> augmented_data_array = form_augmented_data_array(data_array);

    // Setting b to all ones here is equivalent to setting a10 = -1. We can do this without losing
    // generality since any other value would simply be a scaling of all other coefficients, 
    // and since we don't care about true magnitude, it doesn't matter.
    nc::NdArray<double> b = nc::ones<double>({augmented_data_array.shape().rows, 1});

    // Solve the system of linear equations
    //    augmented_data_array * coefficients = b
    // for coefficients. Coefficients then represents the ellipsoid which best fits the data in data_array.
    nc::NdArray<double> coefficients = nc::linalg::lstsq(augmented_data_array, b);

    // Build ellipsoid quadric matrix in regular coordinates. This is matrix A in the sotware design doc.
    nc::NdArray<double> ellipsoid_regular = create_ellipsoid_regular(coefficients);

    // Build the linear part of the ellipsoid equation. This is matrix a in the software design doc.
    nc::NdArray<double> ellipsoid_linear_part{coefficients(0, 6), coefficients(0, 7), coefficients(0, 8)};
    
    // Obtain the centre of the ellipsoid. This will be the bias offset vector.
    nc::NdArray<double> bias_vector = -0.5 * nc::matmul(
        nc::linalg::inv(ellipsoid_regular),
        ellipsoid_linear_part.reshape(3, 1)
    );

    // Move the ellipsoid to the origin of the coordinate system.
    // To do this we first form the quadric matrix in homogeneous coordinates, then move IT to
    // the centre, and use it to discover a scale factor for the quadric in regular coordinates.
    nc::NdArray<double> ellipsoid_homogeneous = create_ellipsoid_homogeneous(coefficients);
    nc::NdArray<double> offset_operator = create_translation_operator_homogeneous(bias_vector);
    nc::NdArray<double> centred_ellipsoid_homogeneous = nc::matmul(nc::matmul(offset_operator.transpose(), ellipsoid_homogeneous), offset_operator);

    // Rescale the ellispsoid quadric matrix in regular coordinates using the last element of the scaled homogeneous form as a scale factor.
    nc::NdArray<double> scaled_ellipsoid_regular_form = ellipsoid_regular * (-1.0 / centred_ellipsoid_homogeneous(3, 3));

    // Take the Cholesky decomposition of ellipsoid_regular. This matrix will transform the ellipsoid
    // onto a sphere, after correcting for bias offset. L is the matrix such that
    //     scaled_ellipsoid_regular_form = L * L.T
    // This is a try-catch because this is the step that may fail if the data matrix was not good enough to
    // fit an ellipse.
    nc::NdArray<double> correction_matrix;
    try
    {
        correction_matrix = nc::linalg::cholesky(scaled_ellipsoid_regular_form).transpose();
    }
    catch (const std::runtime_error& e)
    {
        correction_matrix = nc::eye<double>(3);
        bias_vector = nc::zeros<double>(1, 3);
    }

    std::vector<double> bias_params = {bias_vector(0, 0), bias_vector(0, 1), bias_vector(0, 2)};
    std::vector<std::vector<double>> correction_params = {
        {correction_matrix(0, 0), correction_matrix(0, 1), correction_matrix(0, 2)},
        {correction_matrix(1, 0), correction_matrix(1, 1), correction_matrix(1, 2)},
        {correction_matrix(2, 0), correction_matrix(2, 1), correction_matrix(2, 2)},
    };

    return CalibrationParameters{bias_params, correction_params};
}

/** \brief Reports whether the current series of readings has converged.
 * 
 * Here, converged means that the state estimater has reached a certain confidence level for the
 * readings being fed in, where it as assumed that the readings are all taken with the device
 * stationary in some fixed orientation.
 *
 * \return Boolean indicated whether the state estimater has converged.
 */
bool AccelerometerCalibrator::is_converged() const
{
    return converged;
}

/** \brief Get the currently estimated state for all three accelerometer axes. 
 * \return Current state estimate.
 */
Vector3 AccelerometerCalibrator::get_params() const
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
    kfx = KalmanFilter1D{gravity, initial_data_uncertainty};
    kfy = KalmanFilter1D{gravity, initial_data_uncertainty};
    kfz = KalmanFilter1D{gravity, initial_data_uncertainty};
}

/** \brief Return the interal vector of converged samplings.
 * The vector has size equal to the number of times next_sampling() has been called.
 * \return std::vector of Vector3s.
 */
std::vector<Vector3> AccelerometerCalibrator::get_samplings() const
{
    return samplings;
}

/** \brief Supply a new data reading.
 * \param reading Accelerometer data in units of m/s2.
 * \return Boolean indicating whether convergence has been reached,
 */
bool AccelerometerCalibrator::feed(const Vector3& reading)
{
    kfx.update(reading.x, data_uncertainty);
    kfy.update(reading.y, data_uncertainty);
    kfz.update(reading.z, data_uncertainty);

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
    // 1e6 a sufficiently large number that no real reading will exceed it.
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
bool GyroscopeCalibrator::is_converged() const
{
    return converged;
}

/** \brief Get the currently estimated state for all three gyroscope axes. 
 * \return Current state estimate.
 */
Vector3 GyroscopeCalibrator::get_params() const
{
    return {kfx.est, kfy.est, kfz.est};
}

/** \brief Supply a new data reading.
 * \param reading Gyroscope data in units of rad/s.
 * \return Boolean indicating whether convergence has been reached,
 */
bool GyroscopeCalibrator::feed(const Vector3& reading)
{
    kfx.update(reading.x, data_uncertainty);
    kfy.update(reading.y, data_uncertainty);
    kfz.update(reading.z, data_uncertainty);

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