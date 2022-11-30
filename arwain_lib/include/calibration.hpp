#ifndef _ARWAIN_CALIBRATION_HPP
#define _ARWAIN_CALIBRATION_HPP

#include <NumCpp.hpp>

#include "vector3.hpp"
#include "kalman.hpp"

/** \brief Calibration tool for a 3-axis digital magnetometer. */
class MagnetometerCalibrator
{
    public:
        void feed(const Vector3& reading);
        std::tuple<std::vector<double>, std::vector<std::vector<double>>> solve();
        int get_sphere_coverage_quality() const;
        int get_feed_count() const;

    TESTABLE:
        int sphere_coverage(const std::array<int,100>& region_sample_count) const;
        int sphere_region(const double x, const double y, const double z) const;

    private:
        constexpr static double pi = 3.14159265358979323846;
        int feed_count = 0;
        int sphere_coverage_quality = 0;
        std::array<int, 100> region_sample_count = {0};
        nc::NdArray<double> xyz;
        Vector3 region_sample_value[100] = {{0, 0, 0}};

};

/** \brief Calibration tool for a 3-axis digital accelerometer. */
class AccelerometerCalibrator
{
    private:
        bool converged = false;
        KalmanFilter1D kfx{9.81, 1.0};
        KalmanFilter1D kfy{9.81, 1.0};
        KalmanFilter1D kfz{9.81, 1.0};
        std::vector<Vector3> samplings;

    public:
        bool is_converged();
        Vector3 get_params();
        void next_sampling();
        bool feed(const Vector3& reading);
        std::tuple<Vector3, Vector3> deduce_calib_params();
};

/** \brief Calibration tool for a 3-axis digital gyroscope. */
class GyroscopeCalibrator
{
    private:
        bool converged = false;
        KalmanFilter1D kfx{0, 1};
        KalmanFilter1D kfy{0, 1};
        KalmanFilter1D kfz{0, 1};

    public:
        bool is_converged();
        Vector3 get_params();
        bool feed(const Vector3& reading);
};

#endif
