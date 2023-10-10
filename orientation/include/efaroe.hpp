#ifndef EFAROE_H
#define EFAROE_H

/*
A direct translation of Ross McKenzie's efaroe filter
https://bitbucket.org/arwain/pyori/src/master/pyori/
based on
https://arxiv.org/pdf/1910.00463.pdf
and
https://github.com/manonkok/fastRobustOriEst
and
https://www.sciencedirect.com/science/article/pii/S0888327019303012

*/

#include <array>
#include <iostream>
#include <math.h>

#include <arwain/vector3.hpp>
#include <arwain/quaternion.hpp>

#include "filter.hpp"

namespace arwain
{
    class eFaroe : public OrientationFilter
    {
        public:
            // Constructors.
            eFaroe(
                Quaternion initial_quaternion,
                Vector3 gyro_bias,
                double gyro_error,
                double beta,
                double zeta
            );

            // Update methods.
            void update(
                double timestamp, // time in nanoseconds
                double gx,
                double gy,
                double gz,         // Angular velocities in rads/s
                double ax,
                double ay,
                double az        // Accelerations in m/s2
            )  ;
            void update(
                double timestamp,  // time in nanoseconds
                double gx,
                double gy,
                double gz,        // Angular velocities in rad/s
                double ax,
                double ay,
                double az,          // Accelerations in m/s2
                double mx,
                double my,
                double mz         // Magnetic field in micro tesla
            ) ;

            // Orientation getters.
            double get_w() const;
            double get_x() const;
            double get_y() const;
            double get_z() const;
            double get_pitch() ;
            double get_roll() ;
            double get_yaw() ;
            Quaternion get_q();
            std::array<double, 3> get_euler();

            // Setters
            void set_q(double, double, double, double);
        
        private:
            constexpr static double degrees_per_radian = 57.295779513082320876798154814105;
            constexpr static double radians_per_degree = 0.01745329251994329576923690768489;

            // Current orientation quaternion.
            Quaternion m_quaternion;

            // Gyroscope properties.
            Vector3 m_gyro_bias;
            double m_gyro_error;
            double m_true_error;

            // Magnetic field properties.
            double uk_dip;
            double emf_x_test;
            Vector3 local_emf_test;
            Vector3 emf;
            
            // Filter parameters.
            double m_zeta;
            double m_beta;
            unsigned int conv_count;

            // Flag recording whether Euler angles are up-to-date.
            int computed_angles = 0;

            // Time delta between nth and (n-1)th update.
            double dt = 0;

            // Time of last update.
            double last_read;

            // Euler angles.
            double pitch;
            double roll;
            double yaw;

            // State update methods.
            void update_mag_imu();
            void update_imu();
            void compute_angles();
    };
}

#endif
