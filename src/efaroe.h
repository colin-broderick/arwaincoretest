#ifndef EFAROE_H
#define EFAROE_H

#include <array>
#include <iostream>
#include <math.h>

#include "math_util.h"
#include "quaternions.h"

class eFaroe
{
    public:
        eFaroe(quaternion initial_quaternion, std::array<double, 3> gyro_bias, double gyro_error, int use_mag);
        void update(unsigned long timestamp, double ax, double ay, double az, double gx, double gy, double gz);

        double getW();
        double getX();
        double getY();
        double getZ();

        double getPitch();
        double getRoll();
        double getYaw();

        quaternion getQuat();
        std::array<double, 3> getEuler();
    
    private:
        std::array<double, 3> gyro_bias;
        double gyro_error;
        double true_error;
        double uk_dip;
        std::array<double, 2> emf;
        double zeta;
        double beta;
        unsigned long last_read;
        quaternion initial_quaternion{1, 0, 0, 0};
        unsigned int conv_count;

        double pitch;
        double roll;
        double yaw;

        void update_mag_imu();
        void update_imu();

        quaternion q;
};

#endif
