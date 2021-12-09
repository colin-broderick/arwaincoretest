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
    class Madgwick : public Filter
    {
        private:
            static double invSqrt(double x);
            double m_beta;	// algorithm gain
            double q0;
            double q1;
            double q2;
            double q3;      // quaternion of sensor frame relative to auxiliary frame
            double invSampleFreq;
            double roll;
            double pitch;
            double yaw;
            char anglesComputed;
            void computeAngles();

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
            double getW();
            double getX();
            double getY();
            double getZ();
            double getRoll();
            double getPitch();
            double getYaw();
            double get_beta() const;
            double getRollRadians();
            double getPitchRadians();
            double getYawRadians();

            // Setters
            void setQ(double, double, double, double);
            void set_beta(double beta);
    };
}

#endif
