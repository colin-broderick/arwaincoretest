#ifndef _ARWAIN_LIS3MDL_HPP
#define _ARWAIN_LIS3MDL_HPP

#include <string>
#include <sstream>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <chrono>
#include <thread>

#include "arwain/i2c_interface.hpp"

#include "vector3.hpp"
#include "quaternion.hpp"

template <class I2CDeviceClass>
class LIS3MDL
{
    public: // Types
        enum class FSR
        {
            FSR_4,
            FSR_8,
            FSR_12,
            FSR_16
        };
        enum class ODR
        {
            ODR_5_Hz,
            ODR_10_Hz,
            ODR_20_Hz,
            ODR_40_Hz,
            ODR_80_Hz,
            ODR_155_Hz,
            ODR_300_Hz,
            ODR_500_Hz,
            ODR_1000_Hz
        };

    public: // Methods
        LIS3MDL() = default;
        LIS3MDL(const int i2c_address, const std::string &i2c_bus)
        {
            if (!transport.i2c_init(i2c_address, i2c_bus))
            {
                std::stringstream ss;
                ss << "Could not connect magnetometer: ";
                ss << i2c_bus;
                ss << std::hex;
                ss << " 0x";
                ss << i2c_address;
                throw std::runtime_error{ss.str()};
            }
            soft_reset();
            power_up();
            set_fsr(FSR::FSR_4);
            set_odr(ODR::ODR_300_Hz);
        }

        /** \brief Check chip commication, should return 0x3D. */
        int test_chip()
        {
            uint8_t val;
            transport.i2c_read(ADDR_WHO_AM_I, 1, &val);
            return val;
        }

        /** \brief Read the data from the sensor. The returned values have been calibrated
         * according to any internally set parameters.
         */
        Vector3 read()
        {
            uint8_t read_buffer[6];
            transport.i2c_read(ADDR_OUT_X_L, 6, read_buffer);

            int16_t x_int = (read_buffer[1] << 8) | read_buffer[0];
            int16_t y_int = (read_buffer[3] << 8) | read_buffer[2];
            int16_t z_int = (read_buffer[5] << 8) | read_buffer[4];

            double mag_x = this->fsr_res * x_int;
            double mag_y = this->fsr_res * y_int;
            double mag_z = this->fsr_res * z_int;

            Vector3 v = {
                (mag_x - this->bias.x) * this->scale.x,
                (mag_y - this->bias.y) * this->scale.y,
                (mag_z - this->bias.z) * this->scale.z};

            return {
                v.x + v.y * cross_scale.x + v.z * cross_scale.z,
                v.y + v.z * cross_scale.y,
                v.z};
        }

        /** \brief Measure the current magnetic field vector and compute the rotation required
         * to rotate that vector onto the expected local magnetic field.
         * \return A versor which applies the specified rotation.
         */
        Quaternion read_orientation()
        {
            // static Vector3 mag_target{18.895, -0.361, 45.372}; // The local magnetic field vector.
            static Vector3 mag_target{0.38443, -0.0073448, 0.92312}; // The normalized local magnetic field vector.

            Vector3 mag_measurement = this->read().normalized(); // Normalized magnetic field vector.

            double angle = std::acos(mag_measurement.x * mag_target.x + mag_measurement.y * mag_target.y + mag_measurement.z * mag_target.z); // The angle between the measured field and the local field.

            Vector3 axis = Vector3::cross(mag_measurement, mag_target); // Axis orthogonal to both measured and expected.

            Quaternion quat{
                std::cos(angle / 2.0),
                std::sin(angle / 2.0) * axis.x,
                std::sin(angle / 2.0) * axis.y,
                std::sin(angle / 2.0) * axis.z}; // Rotation operator to rotate measured vector onto expected vector.

            return quat;
        }
        double read_temp()
        {
            uint8_t read_buffer[2];
            transport.i2c_read(ADDR_TEMP_OUT_L, 2, read_buffer);
            int8_t temp_int = (read_buffer[1] << 8) | read_buffer[0];
            return static_cast<double>(temp_int) / 8.0;
        }

        /** \brief Set the calibration of the magnetometer.
         * \param bias_ Vector3 representing hard iron offsets.
         * \param scale_ Vector3 representing scale of axes.
         * \param cross_scale_ Vector3 representing the cross-correlation between axes, in the order xz, yz, xz.
         */
        void set_calibration_parameters(Vector3 bias_, Vector3 scale_, Vector3 cross_scale_)
        {
            this->bias = bias_;
            this->scale = scale_;
            this->cross_scale = cross_scale_;
        }

    TESTABLE : // Methods
        void soft_reset()
        {
            uint8_t val = 1 << 2;
            transport.i2c_write(ADDR_CTRL_REG2, 1, &val);
            sleep_ms(10);
        }
        void power_up()
        {
            uint8_t config;
            transport.i2c_read(ADDR_CTRL_REG3, 1, &config);
            config &= ~(0b00000011);
            transport.i2c_write(ADDR_CTRL_REG3, 1, &config);
            sleep_ms(2);

            // Switch x, y axes to performance mode.
            transport.i2c_read(ADDR_CTRL_REG1, 1, &config);
            config &= ~(3 << 5);
            config |= (3 << 5);
            sleep_ms(2);

            // Switch z axis to performance mode.
            transport.i2c_read(ADDR_CTRL_REG4, 1, &config);
            config &= ~(3 << 2);
            config |= (3 << 2);
            sleep_ms(2);
        }
        void set_fsr(FSR fsr_selection)
        {
            uint8_t config;
            transport.i2c_read(ADDR_CTRL_REG2, 1, &config);
            config &= ~(3 << 5);
            switch (fsr_selection)
            {
                case FSR::FSR_4:
                    this->fsr_res = 4.0 / 27368.0;
                    config |= (0 << 5);
                    break;
                case FSR::FSR_8:
                    this->fsr_res = 8.0 / 27368.0;
                    config |= (1 << 5);
                    break;
                case FSR::FSR_12:
                    this->fsr_res = 12.0 / 27368.0;
                    config |= (2 << 5);
                    break;
                case FSR::FSR_16:
                    this->fsr_res = 16.0 / 27368.0;
                    config |= (3 << 5);
                    break;
            }
            transport.i2c_write(ADDR_CTRL_REG2, 1, &config);
            sleep_ms(2);
        }
        void set_odr(ODR odr_selection)
        {
            uint8_t config;
            transport.i2c_read(ADDR_CTRL_REG1, 1, &config);
            config &= ~(7 << 2);
            switch (odr_selection)
            {
                case ODR::ODR_5_Hz:
                    config |= (3 << 2);
                    break;
                case ODR::ODR_10_Hz:
                    config |= (4 << 2);
                    break;
                case ODR::ODR_20_Hz:
                    config |= (5 << 2);
                    break;
                case ODR::ODR_40_Hz:
                    config |= (6 << 2);
                    break;
                case ODR::ODR_80_Hz:
                    config |= (7 << 2);
                    break;
                case ODR::ODR_155_Hz:
                    config |= (0 << 5);
                    config |= (1 << 1);
                    break;
                case ODR::ODR_300_Hz:
                    config |= (1 << 5);
                    config |= (1 << 1);
                    break;
                case ODR::ODR_500_Hz:
                    config |= (2 << 5);
                    config |= (1 << 1);
                    break;
                case ODR::ODR_1000_Hz:
                    config |= (3 << 5);
                    config |= (1 << 1);
                    break;
            }
            transport.i2c_write(ADDR_CTRL_REG1, 1, &config);
            sleep_ms(2);
        }
        static void sleep_ms(int ms)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }


    TESTABLE: // Attributes
            I2CDeviceClass transport;
            double fsr_res;
            Vector3 bias = {0, 0, 0};
            Vector3 scale = {1, 1, 1};
            Vector3 cross_scale = {0, 0, 0}; // In the order xy, yz, xz.

    TESTABLE: // Statics
            // CHIP ADDRESS -----------------------------------------------------
            static const uint8_t LIS3MDL_I2C_ADDRESS_0 = 0b0011100;
            static const uint8_t LIS3MDL_I2C_ADDRESS_1 = 0b0011110;
            // REGISTER ADDRESSES -----------------------------------------------
            static const uint8_t ADDR_WHO_AM_I = 0x0F;
            static const uint8_t ADDR_CTRL_REG1 = 0x20;
            static const uint8_t ADDR_CTRL_REG2 = 0x21;
            static const uint8_t ADDR_CTRL_REG3 = 0x22;
            static const uint8_t ADDR_CTRL_REG4 = 0x23;
            static const uint8_t ADDR_CTRL_REG5 = 0x24;
            static const uint8_t ADDR_STATUS_REG = 0x27;
            static const uint8_t ADDR_OUT_X_L = 0x28;
            static const uint8_t ADDR_OUT_X_H = 0x29;
            static const uint8_t ADDR_OUT_Y_L = 0x2A;
            static const uint8_t ADDR_OUT_Y_H = 0x2B;
            static const uint8_t ADDR_OUT_Z_L = 0x2C;
            static const uint8_t ADDR_OUT_Z_H = 0x2D;
            static const uint8_t ADDR_TEMP_OUT_L = 0x2E;
            static const uint8_t ADDR_TEMPT_OUT_H = 0x2F;
            static const uint8_t ADDR_INT_CFG = 0x30;
            static const uint8_t ADDR_INT_SRC = 0x31;
            static const uint8_t ADDR_INT_THS_L = 0x32;
            static const uint8_t ADDR_INT_THS_H = 0x33;
            // DEFAULT VALUES --------------------------------------------------
            static const uint8_t DEFAULT_WHO_AM_I = 0b00111101;
            static const uint8_t DEFAULT_CTRL_REG1 = 0b00010000;
            static const uint8_t DEFAULT_CTRL_REG2 = 0b00000000;
            static const uint8_t DEFAULT_CTRL_REG3 = 0b00000011;
            static const uint8_t DEFAULT_CTRL_REG4 = 0b00000000;
            static const uint8_t DEFAULT_CTRL_REG5 = 0b00000000;
            static const uint8_t DEFAULT_INT_CFG = 0b11101000;
            static const uint8_t DEFAULT_INT_SRC = 0b00000000;
            static const uint8_t DEFAULT_INT_THS_L = 0b00000000;
            static const uint8_t DEFAULT_INT_THS_H = 0b00000000;
};

#endif
