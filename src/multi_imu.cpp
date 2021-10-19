#include <chrono>
#include <fstream>

#include "multi_imu.hpp"

Multi_IIM42652::Multi_IIM42652()
{

}

void Multi_IIM42652::read_IMU()
{
    imu1.read_IMU();
    imu2.read_IMU();
    imu3.read_IMU();
   
    this->accelerometer_x = (imu1.accelerometer_x + imu2.accelerometer_x + imu3.accelerometer_x) / 3.0;
    this->accelerometer_y = (imu1.accelerometer_y + imu2.accelerometer_y + imu3.accelerometer_y) / 3.0;
    this->accelerometer_z = (imu1.accelerometer_z + imu2.accelerometer_z + imu3.accelerometer_z) / 3.0;
    this->gyroscope_x = (imu1.gyroscope_x + imu2.gyroscope_x + imu3.gyroscope_x) / 3.0;
    this->gyroscope_y = (imu1.gyroscope_y + imu2.gyroscope_y + imu3.gyroscope_y) / 3.0;
    this->gyroscope_z = (imu1.gyroscope_z + imu2.gyroscope_z + imu3.gyroscope_z) / 3.0;
}

double Multi_IIM42652::read_temperature()
{
    double temperature = 0;
    temperature += imu1.read_temperature();
    temperature += imu2.read_temperature();
    temperature += imu3.read_temperature();
    return temperature / 3.0;
}
