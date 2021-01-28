#ifndef EFAROE_H
#define EFAROE_H

#include <array>
#include <iostream>
#include <math.h>

#include "quaternions.h"

class eFaroe
{
    public:
        eFaroe(quaternion initial_quaternion, std::array<float, 3> gyro_bias, float gyro_error, int use_mag);
        void update(unsigned long timestamp, float ax, float ay, float az, float gx, float gy, float gz);

        float getW();
        float getX();
        float getY();
        float getZ();

        float getPitch();
        float getRoll();
        float getYaw();

        quaternion getQuat();
        std::array<float, 3> getEuler();
    
    private:
        std::array<float, 3> gyro_bias;
        float gyro_error;
        float true_error;
        float uk_dip;
        std::array<float, 2> emf;
        float zeta;
        float beta;
        unsigned long last_read;
        quaternion initial_quaternion{1, 0, 0, 0};
        unsigned int conv_count;

        float pitch;
        float roll;
        float yaw;

        void update_mag_imu();
        void update_imu();

        quaternion q;
};

#endif
