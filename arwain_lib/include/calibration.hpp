#ifndef _ARWAIN_CALIBRATION_HPP
#define _ARWAIN_CALIBRATION_HPP

#include <NumCpp.hpp>

#include "vector3.hpp"
#include "kalman.hpp"

/** \brief Calibration tool for a 3-axis digital magnetometer. */
class MagnetometerCalibrator
{
    public:
        struct CalibrationParameters
        {
            std::vector<double> bias; // 3-vector (x, y, z) of bias offset values.
            std::vector<std::vector<double>> scale; // 3x3 upper triangular matrix of scale parameters.
        };

        constexpr static int total_sphere_regions = 100;
        void feed(const Vector3& reading);
        CalibrationParameters solve();
        int get_sphere_coverage_quality() const;
        int get_feed_count() const;
        std::array<int, 100> get_region_sample_count() const;
        std::array<Vector3, 100> get_region_sample_value() const;

    TESTABLE:
        int sphere_coverage(const std::array<int, total_sphere_regions>& region_sample_count) const;
        int sphere_region(const double x, const double y, const double z) const;
        nc::NdArray<double> form_augmented_data_array(const nc::NdArray<double>& data_array);
        nc::NdArray<double> form_data_array(const std::array<int, total_sphere_regions>& region_sample_counts, const std::array<Vector3, total_sphere_regions>& region_sample_values);
        nc::NdArray<double> create_ellipsoid_homogeneous(const nc::NdArray<double>& params);
        nc::NdArray<double> create_ellipsoid_regular(const nc::NdArray<double>& params);
        nc::NdArray<double> create_translation_operator_homogeneous(const nc::NdArray<double>& vector);
        constexpr static double pi = 3.14159265358979323846;
        int feed_count = 0;
        int sphere_coverage_quality = 0;
        std::array<int, total_sphere_regions> region_sample_count = {0};
        std::array<Vector3, total_sphere_regions> region_sample_value = {{0, 0, 0}};
};

/** \brief Calibration tool for a 3-axis digital accelerometer. */
class AccelerometerCalibrator
{
    constexpr static double gravity = 9.81;
    constexpr static double data_uncertainty = 0.1;
    constexpr static double initial_data_uncertainty = 1.0;

    TESTABLE:
        bool converged = false;
        KalmanFilter1D kfx{gravity, initial_data_uncertainty};
        KalmanFilter1D kfy{gravity, initial_data_uncertainty};
        KalmanFilter1D kfz{gravity, initial_data_uncertainty};
        std::vector<Vector3> samplings;

    public:
        std::vector<Vector3> get_samplings() const;
        bool is_converged() const;
        Vector3 get_params() const;
        void next_sampling();
        bool feed(const Vector3& reading);
        std::tuple<Vector3, Vector3> deduce_calib_params();
};

/** \brief Calibration tool for a 3-axis digital gyroscope. */
class GyroscopeCalibrator
{
    constexpr static double initial_estimate = 0.0;
    constexpr static double initial_estimate_error = 1.0;
    constexpr static double data_uncertainty = 0.02;

    private:
        bool converged = false;
        KalmanFilter1D kfx{initial_estimate, initial_estimate_error};
        KalmanFilter1D kfy{initial_estimate, initial_estimate_error};
        KalmanFilter1D kfz{initial_estimate, initial_estimate_error};

    public:
        Vector3 get_params() const;
        bool is_converged() const;
        bool feed(const Vector3& reading);
};

#endif
