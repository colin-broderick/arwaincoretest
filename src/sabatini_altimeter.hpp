#ifndef _ARWAIN_SABATINI_ALTIMETER_HPP
#define _ARWAIN_SABATINI_ALTIMETER_HPP

#include <eigen3/Eigen/Dense>

namespace arwain::Filters
{
    /** \brief Complementary filter to determine altitude based on accelerometer and pressure sensor measurements.
     * 
     * Based on the derivation on
     * "A Sensor Fusion Method for Tracking Vertical Velocity and Height Based on Inertial and Barometric Altimeter Measurements"
     * in Sensors, 2014, by Sabatini and Genovese
     */
    class SabatiniAltimeter
    {
        private:
            double dt; // Time step between updates, [seconds].
            double gravity; // Local acceleration due to gravity, [m/s2].
            Eigen::Matrix<double, 2, 1> K; // Gain matrix.
            Eigen::Matrix<double, 2, 1> X; // State matrix [position, velocity].
            Eigen::Matrix<double, 2, 2> A; // Position update matrix.
            Eigen::Matrix<double, 2, 2> B; // Noise correction matrix.
            Eigen::Matrix<double, 2, 1> C; // Velocity update matrix.

        public:
            /** \brief Constructor.
             * 
             * If initial parameters are not well specified, the filter may take considerable
             * time to converge.
             * 
             * \param initial_x Initial vertical position [m].
             * \param initial_v Initial vertical velocity [m/s].
             * \param dt The time delta between updates [s].
             * \param sigma_a The standard deviation of accelerometer noise.
             * \param sigma_p The standard deviation in vertical position as determined by barometric pressure.
             */
            SabatiniAltimeter(double initial_x, double initial_v, double dt_, double sigma_a, double sigma_p)
            : dt(dt_)
            {
                X = Eigen::Matrix<double, 2, 1>{initial_x, initial_v};
                A = Eigen::Matrix<double, 2, 2>{{1.0, dt},     {0.0, 1.0}};
                B = Eigen::Matrix<double, 2, 2>{{1.0, dt/2.0}, {0.0, 1.0}};
                C = Eigen::Matrix<double, 2, 1>{dt/2.0, 1.0};
                K = Eigen::Matrix<double, 2, 1>{std::sqrt(2 * sigma_a / sigma_p), sigma_a / sigma_p};
                // TODO According to the paper, K should be negated, but that gives nonsensical results. What gives?
            }

            /** \brief Provide a new accelerometer and pressure altitude to udpate the filter state.
             * 
             * The updated state of the filter is given by
             *      X = A*X + B*K*dt*dx + C*dv
             * where
             *      A is the position update matrix.
             *      B is the noise correction matrix.
             *      C is the velocity update matrix.
             *      K is a gain parameter based on uncertainty in accelerometer and pressure measurements.
             *      dt is the time delta.
             *      dx is the difference between newly-provided pressure altitude and the current state position.
             *      dv is acceleration * dt.
             * 
             * \param vertical_acceleration Vertical acceleration in the world frame.
             * \param pressure_z Vertical position as determined by barometric sensor.
             * \return The new estimate of vertical position.
             */
            double update(double vertical_acceleration, double pressure_altitude)
            {
                X = A * X + B * K * dt * (pressure_altitude - X[0]) + C * vertical_acceleration * dt;
                return X[0];
            }

            double get_position() const
            {
                return X[0];
            }

            double get_velocity() const
            {
                return X[1];
            }
    };
}

#endif
