#include <iostream>
#include <thread>
#include <chrono>

#include "lis3mdl.hpp"

static void sleep_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds{ms});
}

LIS3MDL::LIS3MDL()
{
    soft_reset();
    set_fsr(FSR::FSR_4);
    set_odr(ODR::ODR_40_Hz);
}

void LIS3MDL::set_odr(LIS3MDL::ODR odr_selection)
{
    uint8_t config;
    i2c_read(ADDR_CTRL_REG1, 1, &config);
    config &= ~(7 << 2);
    switch (odr_selection)
    {
        case ODR::ODR_5_Hz:
            config |= (3 << 7);
            break;
        case ODR::ODR_10_Hz:
            config |= (4 << 7);
            break;
        case ODR::ODR_20_Hz:
            config |= (5 << 7);
            break;
        case ODR::ODR_40_Hz:
            config |= (6 << 7);
            break;
        case ODR::ODR_80_Hz:
            config |= (7 << 7);
            break;
        default:
            break;
    }
    i2c_write(ADDR_CTRL_REG1, 1, &config);
    sleep_ms(2);
}

void LIS3MDL::set_fsr(LIS3MDL::FSR fsr_selection)
{
    uint8_t config;
    i2c_read(ADDR_CTRL_REG2, 1, &config);
    config &= ~(3 << 5);
    switch (fsr_selection)
    {
        case FSR::FSR_4:
            this->fsr_res = 8.0 / 65536.0;
            config |= (0 << 5);
            break;
        case FSR::FSR_8:
            this->fsr_res = 16.0 / 65536.0;
            config |= (1 << 5);
            break;
        case FSR::FSR_12:
            this->fsr_res = 24.0 / 65536.0;
            config |= (2 << 5);
            break;
        case FSR::FSR_16:
            this->fsr_res = 32.0 / 65536.0;
            config |= (3 << 5);
            break;
        default:
            break;
    }
    i2c_write(ADDR_CTRL_REG2, 1, &config);
    sleep_ms(2);
}

/** \brief Sets up the I2C file handle and connects to a device on the I2C bus.
 * \param[in] address The address of the device on the I2C bus.
 * \param[in] bus_name The name of the bus to open, e.g. /dev/i2c-1.
 */
void LIS3MDL::i2c_init(const int address, const std::string &bus_name)
{
    //----- OPEN THE I2C BUS -----
    const char *filename = bus_name.c_str();
    if ((this->handle = open(filename, O_RDWR)) < 0)
    {
        //ERROR HANDLING: you can check error number to see what went wrong
        std::cout << "Failed to open I2C bus" << std::endl;
    }

    if (ioctl(this->handle, I2C_SLAVE, address) < 0)
    {
        std::cout << "Failed to connect to I2C address " << std::hex << address << std::endl;
    }
}

/** \brief Reads a given number of bytes from a given register address.
 * \param[in] reg_addr The register address at which to start reading.
 * \param[in] bytes The number of bytes to read.
 * \param[out] buffer The location to store the bytes read from the device.
 * \return SMBUS return code.
 */
int LIS3MDL::i2c_read(int reg_addr, int bytes, uint8_t *buffer)
{
    return i2c_smbus_read_i2c_block_data(this->handle, reg_addr, bytes, buffer);
}

/** \brief Writes a given number of bytes to a specified register address.
 * \param[in] reg_addr The register address to start writing.
 * \param[in] bytes The number of bytes to write.
 * \param[in] buffer The source of the bytes to be written.
 * \return SMBUS return code.
 */
int LIS3MDL::i2c_write(int reg_addr, int bytes, uint8_t *buffer)
{
    return i2c_smbus_write_i2c_block_data(this->handle, reg_addr, bytes, buffer);
}

vector3 LIS3MDL::read()
{
    uint8_t read_buffer[6];
    i2c_read(ADDR_OUT_X_L, 6, read_buffer);

    int8_t x_int = (read_buffer[1] << 8) | read_buffer[0];
    int8_t y_int = (read_buffer[3] << 8) | read_buffer[2];
    int8_t z_int = (read_buffer[5] << 8) | read_buffer[4];

    double mag_x = this->fsr_res * x_int;
    double mag_y = this->fsr_res * y_int;
    double mag_z = this->fsr_res * z_int;

    return {mag_x, mag_y, mag_z};
}

void LIS3MDL::soft_reset()
{
    uint8_t val = 1;
    this->i2c_write(ADDR_CTRL_REG2, 1, &val);
    sleep_ms(10);
}
