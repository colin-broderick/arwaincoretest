#include <chrono>
#include <fstream>

#include "arwain/multi_imu.hpp"

#include "vector3.hpp"

Multi_IIM42652::Multi_IIM42652()
{

}

void Multi_IIM42652::read_imu()
{
    auto [accel1, gyro1] = imu1.read_imu();
    auto [accel2, gyro2] = imu2.read_imu();
    auto [accel3, gyro3] = imu3.read_imu();
   
    this->accelerometer_x = (accel1.x + accel2.x + accel3.x) / 3.0;
    this->accelerometer_y = (accel1.y + accel2.y + accel3.y) / 3.0;
    this->accelerometer_z = (accel1.z + accel2.z + accel3.z) / 3.0;
    this->gyroscope_x = (gyro1.x + gyro2.x + gyro3.x) / 3.0;
    this->gyroscope_y = (gyro1.y + gyro2.y + gyro3.y) / 3.0;
    this->gyroscope_z = (gyro1.z + gyro2.z + gyro3.z) / 3.0;
}

double Multi_IIM42652::read_temperature()
{
    double temperature = 0;
    temperature += imu1.read_temperature();
    temperature += imu2.read_temperature();
    temperature += imu3.read_temperature();
    return temperature / 3.0;
}
