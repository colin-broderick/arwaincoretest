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

    TESTABLE:
        int sphere_coverage(const std::array<int,100>& region_sample_count) const;
        int sphere_region(const double x, const double y, const double z) const;

    private:
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
        AccelerometerCalibrator()
        {

        }

        bool is_converged()
        {
            return converged;
        }

        Vector3 get_params()
        {
            return {kfx.est, kfy.est, kfz.est};
        }

        void next_sampling()
        {
            converged = false;
            samplings.push_back({kfx.est, kfy.est, kfz.est});
            kfx = KalmanFilter1D{9.81, 1.0};
            kfy = KalmanFilter1D{9.81, 1.0};
            kfz = KalmanFilter1D{9.81, 1.0};
        }

        bool feed(const Vector3& reading)
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

        std::tuple<Vector3, Vector3> deduce_calib_params()
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
        GyroscopeCalibrator()
        {
        }

        bool is_converged()
        {
            return converged;
        }

        Vector3 get_params()
        {
            return {kfx.est, kfy.est, kfz.est};
        }

        bool feed(const Vector3& reading)
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
};

#endif
