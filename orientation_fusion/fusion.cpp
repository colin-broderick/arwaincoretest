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
        ImuData imu1_data = imu1.read_IMU();
        filter1.update(
            imu1_data.gyro.x,
            imu1_data.gyro.y,
            imu1_data.gyro.z,
            imu1_data.acce.x,
            imu1_data.acce.y,
            imu1_data.acce.z
        );

        ImuData imu2_data = imu2.read_IMU();
        filter2.update(
            imu2_data.gyro.x,
            imu2_data.gyro.y,
            imu2_data.gyro.z,
            imu2_data.acce.x,
            imu2_data.acce.y,
            imu2_data.acce.z
        );

        ImuData imu3_data = imu3.read_IMU();
        filter3.update(
            imu3_data.gyro.x,
            imu3_data.gyro.y,
            imu3_data.gyro.z,
            imu3_data.acce.x,
            imu3_data.acce.y,
            imu3_data.acce.z
        );

        std::cout << "FILTER1: " << filter1.getW() << " " << filter1.getX() << " " << filter1.getY() << " " << filter1.getZ() << "\n";
        std::cout << "FILTER2: " << filter2.getW() << " " << filter2.getX() << " " << filter2.getY() << " " << filter2.getZ() << "\n";
        std::cout << "FILTER3: " << filter3.getW() << " " << filter3.getX() << " " << filter3.getY() << " " << filter3.getZ() << "\n";
        std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }

    return 0;
}
