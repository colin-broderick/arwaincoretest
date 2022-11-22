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
        int get_sphere_coverage_quality();

    private:
        int feed_count = 0;
        int sphere_coverage_quality = 0;
        int region_sample_count[100] = {0};
        nc::NdArray<double> xyz;
        Vector3 region_sample_value[100] = {{0, 0, 0}};

};

/** \brief Calibration tool for a 3-axis digital accelerometer. */
class AccelerometerCalibrator
{

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
