#ifndef GREEVE_MULTI_IMU_HPP
#define GREEVE_MULTI_IMU_HPP

#include "IMU_IIM42652_driver.hpp"

class Multi_IIM42652
{
    public:
        Multi_IIM42652();
        void read_IMU();
        double read_temperature();
        double accelerometer_x, accelerometer_y, accelerometer_z, gyroscope_x, gyroscope_y, gyroscope_z;
    
    private:
        IMU_IIM42652 imu1{0x68, "/dev/i2c-1"};
        IMU_IIM42652 imu2{0x69, "/dev/i2c-1"};
        IMU_IIM42652 imu3{0x68, "/dev/i2c-4"};
};

#endif
