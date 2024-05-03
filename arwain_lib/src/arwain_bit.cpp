#include <arwain/input_parser.hpp>
#include <arwain/devices/iim42652.hpp>
#include <arwain/devices/rfm95w.hpp>
#include <arwain/devices/lis3mdl.hpp>
#include <arwain/devices/bmp384.hpp>

#include "arwain/i2c_interface.hpp"
#include "arwain/arwain.hpp"
#include "arwain/arwain_bit.hpp"
#include "arwain/configuration.hpp"

ArwainBIT arwain_bit(int argc, char **argv)
{
    // If configuration file cannot be read, no further tests can be executed and
    // system operation is not possible.
    try
    {
        arwain::InputParser input{argc, argv};
        arwain::config = arwain::Configuration{input.get_cmd_option("--conf")};
        arwain::config.read_from_file();
    }
    catch (const std::exception &e)
    {
        return ArwainBIT::CONFIG_FAILED;
    }

    // Test IMUs.
    IIM42652<PlatformI2CDevice> imu1{arwain::config.imu1_address, arwain::config.imu1_bus};
    if (imu1.whoami() != IIM42652<PlatformI2CDevice>::chip_id)
    {
        return ArwainBIT::IMU1_FAILED;
    }
    IIM42652<PlatformI2CDevice> imu2{arwain::config.imu2_address, arwain::config.imu2_bus};
    if (imu2.whoami() != IIM42652<PlatformI2CDevice>::chip_id)
    {
        return ArwainBIT::IMU2_FAILED;
    }
    IIM42652<PlatformI2CDevice> imu3{arwain::config.imu3_address, arwain::config.imu3_bus};
    if (imu3.whoami() != IIM42652<PlatformI2CDevice>::chip_id)
    {
        return ArwainBIT::IMU3_FAILED;
    }

    // Test pressure
    BMP384<PlatformI2CDevice> pressure_sensor{arwain::config.pressure_address, arwain::config.pressure_bus};
    auto chip_id = pressure_sensor.whoami();
    bool condition = (chip_id == BMP384<PlatformI2CDevice>::BMP384_chip_id) || (chip_id == BMP384<PlatformI2CDevice>::BMP390_chip_id);
    if (!condition)
    {
        return ArwainBIT::PRESSURE_FAILED;
    }

    // Test magnetometer
    LIS3MDL<PlatformI2CDevice> magnetometer{arwain::config.magn_address, arwain::config.magn_bus};
    if (magnetometer.whoami() != LIS3MDL<PlatformI2CDevice>::chip_id)
    {
        return ArwainBIT::MAG_FAILED;
    }

    // Test LoRa
    RFM95W<LinuxSpiDevice> lora{arwain::config.lora_address, AsReceiver::No};
    if (lora.test_chip() != RFM95W<LinuxSpiDevice>::chip_id)
    {
        return ArwainBIT::LORA_FAILED;
    }

    return ArwainBIT::ALL_OK;
}
