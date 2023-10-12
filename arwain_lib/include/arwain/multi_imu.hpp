// TODO: The initialization of the IMUs should be in the constructor, perhaps with addresses/busses as params.

#ifndef _ARWAIN_MULTI_IMU_HPP
#define _ARWAIN_MULTI_IMU_HPP

#include <arwain/devices/iim42652.hpp>

#include "i2c_interface.hpp"

class Multi_IIM42652
{
    public:
        Multi_IIM42652();
        void read_imu();
        double read_temperature();
        double accelerometer_x, accelerometer_y, accelerometer_z, gyroscope_x, gyroscope_y, gyroscope_z;
    
    private:
        IIM42652<I2CDEVICEDRIVER> imu1{0x68, "/dev/i2c-1"};
        IIM42652<I2CDEVICEDRIVER> imu2{0x69, "/dev/i2c-1"};
        IIM42652<I2CDEVICEDRIVER> imu3{0x68, "/dev/i2c-4"};
};

#endif
