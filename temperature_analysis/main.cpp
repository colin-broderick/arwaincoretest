#include <thread>
#include <chrono>
#include <fstream>

#include "IMU_IIM42652_driver.hpp"

int main()
{
    IMU_IIM42652 imu{0x68, "/dev/i2c-1"};

    std::ofstream output{"temperature_output.log"};
    output << "# temp ax ay az gx gy gz" << std::endl;

    std::chrono::milliseconds interval{10};

    while (true)
    {
        imu.read_IMU();
        imu.read_temperature();
        output << imu.temperature << " "
               << imu.accelerometer_x << " "
               << imu.accelerometer_y << " "
               << imu.accelerometer_z << " "
               << imu.gyroscope_x << " "
               << imu.gyroscope_y << " "
               << imu.gyroscope_y
               <<  std::endl;
        std::this_thread::sleep_for(interval);
    }
}
