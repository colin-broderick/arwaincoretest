#include "IMU_IIM42652_driver.hpp"
#include "madgwick.hpp"

#include <thread>
#include <chrono>

int main()
{
    arwain::Madgwick filter1;
    arwain::Madgwick filter2;
    arwain::Madgwick filter3;

    IMU_IIM42652 imu1{0x68, "/dev/i2c-1"};
    IMU_IIM42652 imu2{0x69, "/dev/i2c-1"};
    IMU_IIM42652 imu3{0x68, "/dev/i2c-4"};

    while (true)
    {
        imu1.read_IMU();
        filter1.update(
            imu1.gyroscope_x,
            imu1.gyroscope_y,
            imu1.gyroscope_z,
            imu1.accelerometer_x,
            imu1.accelerometer_y,
            imu1.accelerometer_z
        );

        imu2.read_IMU();
        filter2.update(
            imu2.gyroscope_x,
            imu2.gyroscope_y,
            imu2.gyroscope_z,
            imu2.accelerometer_x,
            imu2.accelerometer_y,
            imu2.accelerometer_z
        );

        imu3.read_IMU();
        filter3.update(
            imu3.gyroscope_x,
            imu3.gyroscope_y,
            imu3.gyroscope_z,
            imu3.accelerometer_x,
            imu3.accelerometer_y,
            imu3.accelerometer_z
        );

        std::cout << "FILTER1: " << filter1.getW() << " " << filter1.getX() << " " << filter1.getY() << " " << filter1.getZ() << "\n";
        std::cout << "FILTER2: " << filter2.getW() << " " << filter2.getX() << " " << filter2.getY() << " " << filter2.getZ() << "\n";
        std::cout << "FILTER3: " << filter3.getW() << " " << filter3.getX() << " " << filter3.getY() << " " << filter3.getZ() << "\n";
        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }

    return 0;
}
