#ifndef _ARWAIN_LIS3MDL_HPP
#define _ARWAIN_LIS3MDL_HPP

#include <string>
#include <sys/ioctl.h>
#include <fcntl.h>
extern "C"
{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include "vector3.hpp"

class LIS3MDL
{
    public: // Types
        enum class FSR { FSR_4, FSR_8, FSR_12, FSR_16 };
        enum class ODR { ODR_5_Hz, ODR_10_Hz, ODR_20_Hz, ODR_40_Hz, ODR_80_Hz };
    
    public: // Methods
        LIS3MDL(const int i2c_address, const std::string& i2c_bus);
        int test_chip();
        vector3 read();
        double read_temp();

    private: // Methods
        void soft_reset();
        void power_up();
        void set_fsr(FSR fsr_selection);
        void set_odr(ODR odr_selection);


        void i2c_init(const int address, const std::string &bus_name);
        int i2c_read(int reg_addr, int bytes, uint8_t *buffer);
        int i2c_write(int reg_addr, int bytes, uint8_t *buffer);

    private: // Attributes
        int handle = 0;
        double fsr_res;

    private: // Statics
        // CHIP ADDRESS -----------------------------------------------------
        static const uint8_t LIS3MDL_I2C_ADDRESS_0 = 0b0011100;
        static const uint8_t LIS3MDL_I2C_ADDRESS_1 = 0b0011110;
        // REGISTER ADDRESSES -----------------------------------------------
        static const uint8_t ADDR_WHO_AM_I    = 0x0F;
        static const uint8_t ADDR_CTRL_REG1   = 0x20;
        static const uint8_t ADDR_CTRL_REG2   = 0x21;
        static const uint8_t ADDR_CTRL_REG3   = 0x22;
        static const uint8_t ADDR_CTRL_REG4   = 0x23;
        static const uint8_t ADDR_CTRL_REG5   = 0x24;
        static const uint8_t ADDR_STATUS_REG  = 0x27;
        static const uint8_t ADDR_OUT_X_L     = 0x28;
        static const uint8_t ADDR_OUT_X_H     = 0x29;
        static const uint8_t ADDR_OUT_Y_L     = 0x2A;
        static const uint8_t ADDR_OUT_Y_H     = 0x2B;
        static const uint8_t ADDR_OUT_Z_L     = 0x2C;
        static const uint8_t ADDR_OUT_Z_H     = 0x2D;
        static const uint8_t ADDR_TEMP_OUT_L  = 0x2E;
        static const uint8_t ADDR_TEMPT_OUT_H = 0x2F;
        static const uint8_t ADDR_INT_CFG     = 0x30;
        static const uint8_t ADDR_INT_SRC     = 0x31;
        static const uint8_t ADDR_INT_THS_L   = 0x32;
        static const uint8_t ADDR_INT_THS_H   = 0x33;
        // DEFAULT VALUES --------------------------------------------------
        static const uint8_t DEFAULT_WHO_AM_I  = 0b00111101;
        static const uint8_t DEFAULT_CTRL_REG1 = 0b00010000;
        static const uint8_t DEFAULT_CTRL_REG2 = 0b00000000;
        static const uint8_t DEFAULT_CTRL_REG3 = 0b00000011;
        static const uint8_t DEFAULT_CTRL_REG4 = 0b00000000;
        static const uint8_t DEFAULT_CTRL_REG5 = 0b00000000;
        static const uint8_t DEFAULT_INT_CFG   = 0b11101000;
        static const uint8_t DEFAULT_INT_SRC   = 0b00000000;
        static const uint8_t DEFAULT_INT_THS_L = 0b00000000;
        static const uint8_t DEFAULT_INT_THS_H = 0b00000000;
};

#endif
