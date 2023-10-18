//=============================================================================================
// MadgwickAHRS.c
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=============================================================================================
//
// This file modified by Greeve to bring comments in line with the rest of the project,
// and to match APIs with another library, but core functionality is not changed.
//
//=============================================================================================

#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include "filter.hpp"

namespace arwain
{
    class Madgwick : public OrientationFilter
    {

        private:
            double beta;	// algorithm gain
            double w;
            double x;
            double y;
            double z;      // quaternion of sensor frame relative to auxiliary frame
            double inverse_sample_frequency;
            double roll;
            double pitch;
            double yaw;
            bool angles_computed;
            
        private:
            constexpr static double sample_frequency_default = 512.0;
            constexpr static double beta_default = 0.1;
            void compute_angles();
            static double inverse_sqrt(double x);

        public:
            // Constructors
            Madgwick(void);
            Madgwick(double sample_frequency, double beta);

            // General methods
            void update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
            void update(double gx, double gy, double gz, double ax, double ay, double az);
            void update(double timestamp, double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz);
            void update(double timestamp, double gx, double gy, double gz, double ax, double ay, double az);

            // Getters
            double get_w() const;
            double get_x() const;
            double get_y() const;
            double get_z() const;
            double get_roll();
            double get_pitch();
            double get_yaw();
            double get_beta() const;
            double get_roll_radians();
            double get_pitch_radians();
            double get_yaw_radians();
            double get_inverse_sample_frequency() const;
            double get_dt() const;
            double get_frequency() const;
            bool angles_updated() const;

            // Setters
            void set_q(double w, double x, double y, double z);
            void set_beta(double beta);
    };
}

#endif
