#ifndef _GREEVE_I2C_INTERFACE
#define _GREEVE_I2C_INTERFACE

#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <cstring>

extern "C"
{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

/** \brief I2C interface defintion. */
class I_I2C
{
    public:
        /** \brief Reads a given number of bytes from a given register address.
         * \param[in] reg_addr The register address at which to start reading.
         * \param[in] bytes The number of bytes to read.
         * \param[out] buffer The location to store the bytes read from the device.
         * \return SMBUS return code.
         */
        virtual int i2c_read(int reg_addr, int bytes, uint8_t *buffer) = 0;

        /** \brief Writes a given number of bytes to a specified register address.
         * \param[in] reg_addr The register address to start writing.
         * \param[in] bytes The number of bytes to write.
         * \param[in] buffer The source of the bytes to be written.
         * \return SMBUS return code.
         */
        virtual int i2c_write(int reg_addr, int bytes, uint8_t *buffer) = 0;

        /** \brief Sets up the I2C file handle and connects to a device on the I2C bus.
         * \param[in] address The address of the device on the I2C bus.
         * \param[in] bus_name The name of the bus to open, e.g. /dev/i2c-1.
         * \return Boolean; true for successful opening of file handle, false for failure.
         */
        virtual bool i2c_init(const int address, const std::string &bus_name) = 0;
};
#if UNIT_TESTS
class MockI2CDevice : public I_I2C
{
    /*  (void) casts are present to suppress unused parameter warnings. */
    public:
        int i2c_read(int reg_addr, int bytes, uint8_t *buffer) override
        {
            std::memset(buffer, 0, bytes);
            (void)reg_addr;
            return 0;
        }
        int i2c_write(int reg_addr, int bytes, uint8_t *buffer) override
        {
            std::memset(buffer, 0, bytes);
            (void)reg_addr;
            return 0;
        }
        [[nodiscard]] bool i2c_init(const int address, const std::string &bus_name) override
        {
            if (address == -1 || bus_name == "fail_bus")
            {
                return false;
            }
            return true;
        }
};

#else

class LinuxSmbusI2CDevice : public I_I2C
{
    private:
        int handle;
    
    public:
        int i2c_read(int reg_addr, int bytes, uint8_t *buffer) override
        {
            return i2c_smbus_read_i2c_block_data(this->handle, reg_addr, bytes, buffer);
        }

        int i2c_write(int reg_addr, int bytes, uint8_t *buffer) override
        {
            return i2c_smbus_write_i2c_block_data(this->handle, reg_addr, bytes, buffer);
        }

        [[nodiscard]] bool i2c_init(const int address, const std::string &bus_name) override
        {
            //----- OPEN THE I2C BUS -----
            const char *filename = bus_name.c_str();
            if ((this->handle = open(filename, O_RDWR)) < 0)
            {
                // ERROR HANDLING: you can check error number to see what went wrong
                std::cout << "Failed to open I2C bus" << std::endl;
                return false;
            }

            if (ioctl(this->handle, I2C_SLAVE, address) < 0)
            {
                std::cout << "Failed to connect to I2C address " << std::hex << address << std::endl;
                return false;
            }

            return true;
        }
};

#endif

#endif
