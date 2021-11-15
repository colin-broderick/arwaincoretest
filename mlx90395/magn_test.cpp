#include <chrono>
#include <thread>
#include <iostream>

#include "mlx90395.hpp"

int main()
{
    MLX90395 magnetometer{0x0C, "/dev/i2c-4"};

    // auto mag = magnetometer.read();

    while (true)
    {
        auto mag = magnetometer.read();
        std::cout << mag << "     " << mag.magnitude() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds{500});
    }
    return 0;
}