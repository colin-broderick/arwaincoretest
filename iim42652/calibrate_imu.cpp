#include <sstream>

#include "IMU_IIM42652_driver.hpp"
#include "input_parser.hpp"

int main(int argc, char* argv[])
{
    // Check appropriate arguments passed.
    InputParser input{argc, argv};
    if (!input.contains("-address") || !input.contains("-bus"))
    {
        std::cout << "Specify I2C bus and address" << std::endl;
        return 0;
    }

    // Parse I2C address and bus from command line args.
    int address;
    std::string bus = input.getCmdOption("-bus");
    std::stringstream(input.getCmdOption("-address")) >> std::hex >> address;

    // Create IMU and run calibration methods.
    IMU_IIM42652 imu{address, bus};
    imu.calibrate_gyroscope();
    imu.calibrate_accelerometer();
    return 0;
}
