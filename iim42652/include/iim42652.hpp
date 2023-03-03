#ifndef GREEVE_IIM42652_HPP
#define GREEVE_IIM42652_HPP

#include <thread>
#include <iostream>
#include <map>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <eigen3/Eigen/Dense>
extern "C"
{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include "i2c_interface.hpp"
#include "vector3.hpp"

// CONSTANTS ===================================================================

#define GRAVITY 9.8067
#define PI_DIVIDED_BY_180 0.01745329251994329576923690768489

// CONFIG_MASKS ================================================================

#define CONFIG_SOFT_RESET     0b00000001
#define CONFIG_ENABLE_ACCEL   0b00000011
#define CONFIG_ENABLE_GYRO    0b00001100
#define CONFIG_ENABLE_TMPRTR  0b00000000
#define CONFIG_DISABLE_TMPRTR 0b01000000

#define ACCEL_200HZ  0b00000111
#define ACCEL_FSR_2G 0b01100000
#define ACCEL_FSR_16G 0b00000000

#define GYRO_200HZ      0b00000111
#define GYRO_FSR_2000   0b00000000
#define GYRO_FSR_1000   0b00100000
#define GYRO_FSR_500    0b01000000
#define GYRO_FSR_250    0b01100000
#define GYRO_FSR_125    0b10000000
#define GYRO_FSR_62_5   0b10100000
#define GYRO_FSR_31_25  0b11000000
#define GYRO_FSR_15_625 0b11100000

#define GYRO_UI_FILTER_ORD_3RD 0b00001000

// DATA REGISTERS ================================================================

#define ADDR_DEVICE_CONFIG 0x11
#define ADDR_PWR_MGMT0 0x4E

#define ADDR_ACCEL_CONFIG0 0x50
#define ADDR_GYRO_CONFIG0 0x4F
#define ADDR_GYRO_CONFIG1 0x51

#define ADDR_TMPRTR 0x1D

#define ADDR_ALL_IMU 0x1F
#define ADDR_ACCEL_X1 0x1F
#define ADDR_ACCEL_X0 0x20
#define ADDR_ACCEL_Y1 0x21
#define ADDR_ACCEL_Y0 0x22
#define ADDR_ACCEL_Z1 0x23
#define ADDR_ACCEL_Z0 0x24

#define ADDR_GYRO_X1 0x25
#define ADDR_GYRO_X0 0x26
#define ADDR_GYRO_Y1 0x27
#define ADDR_GYRO_Y0 0x28
#define ADDR_GYRO_Z1 0x29
#define ADDR_GYRO_Z0 0x2A

// DATA CONVERSION FACTORS =========================================================

#define ACCEL_RES_2G 0.00006103515625
#define ACCEL_RES_4G 0.0001220703125
#define ACCEL_RES_8G 0.000244140625
#define ACCEL_RES_16G 0.00048828125

#define GYRO_RES_15_625 4.76837158203125e-4
#define GYRO_RES_31_25 9.5367431640625e-4
#define GYRO_RES_62_5 0.0019073486328125
#define GYRO_RES_125 0.003814697265625
#define GYRO_RES_250 0.00762939453125
#define GYRO_RES_500 0.0152587890625
#define GYRO_RES_1000 0.030517578125
#define GYRO_RES_2000 0.06103515625

class Vector3;
class ImuData;

template <class I2CDeviceClass>
class IIM42652
{
    TESTABLE:
        I2CDeviceClass transport;
    public:
        IIM42652() = default;

        ~IIM42652() = default;

        /** \brief Constructor.
         * \param[in] bus_address The I2C address of the device.
         * \param[in] bus_name The bus on which the device is found, e.g. "/dev/i2c-1".
         */
        IIM42652(int bus_address, const std::string &bus_name)
        : address_(bus_address), bus_name_(bus_name)
        {
            // Default configuration.
            unsigned char accel_config = ACCEL_200HZ | ACCEL_FSR_16G;
            unsigned char gyro_config = GYRO_200HZ | GYRO_FSR_1000;

            if (!transport.i2c_init(bus_address, bus_name))
            {
                std::stringstream ss;
                ss << "Could not connect IMU: ";
                ss << bus_name;
                ss << std::hex;
                ss << " 0x";
                ss << bus_address;
                throw std::runtime_error{ss.str()};
            }
            soft_reset();
            std::this_thread::sleep_for(std::chrono::milliseconds{2});
            imu_config(gyro_config, accel_config);
            set_resolutions(ACCEL_RES_16G, GYRO_RES_1000);
            enable();
            std::this_thread::sleep_for(std::chrono::milliseconds{25});
        }

        /** \brief Read in current config, then write back with reset bit set. Wait 2 ms after reset. */
        void soft_reset()
        {
            uint8_t buffer = CONFIG_SOFT_RESET;
            transport.i2c_write(ADDR_DEVICE_CONFIG, 1, &buffer);
        }
        
        /** \brief Writes FSR and ODR settings to device registers.
         * \param gyro_config Four bytes of gyro ODR | four bytes of gyro FSR.
         * \param gyro_config Four bytes of accel ODR | four bytes of accel FSR.
        */
        int imu_config(uint8_t gyro_config, uint8_t accel_config)
        {
            int ret_code = transport.i2c_write(ADDR_GYRO_CONFIG0, 1, &gyro_config);
            if (ret_code > 0)
            {
                return ret_code;
            }
            ret_code = transport.i2c_write(ADDR_ACCEL_CONFIG0, 1, &accel_config);
            if (ret_code > 0)
            {
                return ret_code;
            }

            // Set internal gyro filter order.
            uint8_t val = GYRO_UI_FILTER_ORD_3RD | 0b00000010;
            ret_code = transport.i2c_write(ADDR_GYRO_CONFIG1, 1, &val);
            if (ret_code > 0)
            {
                return ret_code;
            }

            return ret_code;
        }

        /** \brief Sets accel/gyro conversion factors to match the FSR configuration.
         * \param accel Conversion factor from raw accel data to double representation in units of g.
         * \param gyro Conversion factor from raw gyro data to double representation in units of degrees per second.
         */
        void set_resolutions(double accel, double gyro)
        {
            accel_resolution = accel;
            gyro_resolution = gyro;
        }

        /** \brief Fetches raw IMU data and applies conversion factors to get correct units. Alters state. */
        ImuData read_imu()
        {
            read_imu_raw_data();

            // Store new accelerometer reading.
            accelerometer_x = (accelerometer_x * accel_resolution * GRAVITY - accel_bias_x) * accel_scale_x;
            accelerometer_y = (accelerometer_y * accel_resolution * GRAVITY - accel_bias_y) * accel_scale_y;
            accelerometer_z = (accelerometer_z * accel_resolution * GRAVITY - accel_bias_z) * accel_scale_z;

            // Store new gyroscope reading.
            gyroscope_x = gyroscope_x * gyro_resolution * PI_DIVIDED_BY_180 - gyro_bias_x;
            gyroscope_y = gyroscope_y * gyro_resolution * PI_DIVIDED_BY_180 - gyro_bias_y;
            gyroscope_z = gyroscope_z * gyro_resolution * PI_DIVIDED_BY_180 - gyro_bias_z;

            // Automatically update gyroscope bias when device static.
            if (auto_calib_enabled_)
            {
                this->update_gyro_bias();
            }

            return {
                {accelerometer_x, accelerometer_y, accelerometer_z},
                {gyroscope_x, gyroscope_y, gyroscope_z}
            };
        }
        /** \brief Read the temperature from the IMU, update the state of this
         * object with the new value.
         * \return The temperature of the sensor in degrees centigrade.
         */
        double read_temperature()
        {
            uint8_t bf[2];
            transport.i2c_read(ADDR_TMPRTR, 2, bf);

            int16_t temp = 0;
            temp = (bf[0] << 8) | bf[1];
            this->temperature = (temp / 132.48) + 25; // Conversion factor from datasheet.

            return this->temperature;
        }
        /** \brief Set the values of the gyroscope bias offsets, subtracted from the gyro readings before returning.
         */
        void set_gyro_bias(double x, double y, double z)
        {
            gyro_bias_x = x;
            gyro_bias_y = y;
            gyro_bias_z = z;
        }

        void set_accel_bias(double x, double y, double z)
        {
            accel_bias_x = x;
            accel_bias_y = y;
            accel_bias_z = z;
        }

        void set_accel_scale(double x, double y, double z)
        {
            accel_scale_x = x;
            accel_scale_y = y;
            accel_scale_z = z;
        }

        /** \brief Set the gain on the EWMA filter which adjusts gyro bias.
         * \param speed Value between 0 and 1; update step is prev * speed + new * (1 - speed).
         */
        void set_correction_speed(double speed)
        {
            if (speed < 0)
            {
                correction_speed = 0;
            }
            else if (speed > 1)
            {
                correction_speed = 1;
            }
            else
            {
                correction_speed = speed;
            }
        }

        void enable_auto_calib()
        {
            auto_calib_enabled_ = true;
        }

        void enable_auto_calib(double threshold)
        {
            this->auto_calib_threshold = threshold;
            this->enable_auto_calib();
        }

        void disable_auto_calib()
        {
            auto_calib_enabled_ = false;
        }

        Vector3 get_gyro_calib()
        {
            return {gyro_bias_x, gyro_bias_y, gyro_bias_z};
        }
        double get_gyro_calib_x()
        {
            return gyro_bias_x;
        }

        double get_gyro_calib_y()
        {
            return gyro_bias_y;
        }
        double get_gyro_calib_z()
        {
            return gyro_bias_z;
        }
        int get_address() const
        {
            return address_;
        }
        std::string get_bus() const
        {
            return bus_name_;
        }
        /** \brief Gives the current auto calibration state. 
         * \return bool indiciating whether autocalibration is enabled; true for on, false for off.
         */
        bool auto_calib_enabled() const
        {
            return auto_calib_enabled_;
        }


    TESTABLE:
        double accel_resolution = 0;
        double gyro_resolution = 0;
        int handle = 0;
        double temperature = 0;

        // Calibration parameters.
        double gyro_bias_x = 0;
        double gyro_bias_y = 0;
        double gyro_bias_z = 0;
        double accel_bias_x = 0;
        double accel_bias_y = 0;
        double accel_bias_z = 0;
        double accel_scale_x = 1;
        double accel_scale_y = 1;
        double accel_scale_z = 1;

        // Most recent reading lives here.
        double gyroscope_x = 0;
        double gyroscope_y = 0;
        double gyroscope_z = 0;
        double accelerometer_x = 0;
        double accelerometer_y = 0;
        double accelerometer_z = 0;
        
        double auto_calib_timer = 0;
        int calib_time = 200;
        double auto_calib_threshold = 0.025;
        bool auto_calib_enabled_ = false;
        double correction_speed = 0.995;

        int address_ = 0;
        std::string bus_name_;

    TESTABLE:
        /** \brief Set register values to enable temperature sensor, accelerometer, gyroscope. */
        void enable()
        {
            uint8_t buffer = CONFIG_ENABLE_ACCEL | CONFIG_ENABLE_GYRO | CONFIG_ENABLE_TMPRTR;
            transport.i2c_write(ADDR_PWR_MGMT0, 1, &buffer);
        }

        /** \brief Reads the registers containing raw accelerometer and gyroscope data. */
        void read_imu_raw_data()
        {
            uint8_t buffer[12];
            int ret_code;
            int16_t val = 0;

            ret_code = transport.i2c_read(ADDR_ALL_IMU, 12, buffer);
            if (ret_code > 0)
            {
                //add condition for error?
            }

            // After this block, accel_x, etc, are the double representations of the 16bit ints read from the device.
            val = buffer[1] | (buffer[0] << 8);
            this->accelerometer_x = static_cast<double>(val);

            val = buffer[3] | (buffer[2] << 8);
            this->accelerometer_y = static_cast<double>(val);

            val = buffer[5] | (buffer[4] << 8);
            this->accelerometer_z = static_cast<double>(val);

            val = buffer[7] | (buffer[6] << 8);
            this->gyroscope_x = static_cast<double>(val);

            val = buffer[9] | (buffer[8] << 8);
            this->gyroscope_y = static_cast<double>(val);

            val = buffer[11] | (buffer[10] << 8);
            this->gyroscope_z = static_cast<double>(val);
        }



        /** \brief Automatically update the gyroscope bias when the device is detected to be stationary.
         * \return Boolean false if no change to calib parameters. True if calib parameters changed.
         */
        bool update_gyro_bias()
        {
            // If any gyro readings are above the auto calib threshold, reset the timer.
            if (std::abs(this->gyroscope_x) > auto_calib_threshold || std::abs(this->gyroscope_y) > auto_calib_threshold || std::abs(this->gyroscope_z) > this->auto_calib_threshold)
            {
                this->auto_calib_timer = 0;
                return false;;
            }

            // If the minimum static time has not yet elapsed, increment the timer.
            if (this->auto_calib_timer < this->calib_time)
            {
                this->auto_calib_timer++;
                return false;
            }

            // If not gyro reading breaks the threshold, and the timer has elapsed, use the new reading to update the bias.
            // And knock the timer back a bit.
            this->gyro_bias_x = this->gyro_bias_x * this->correction_speed + (this->gyroscope_x + gyro_bias_x) * (1.0 - this->correction_speed);
            this->gyro_bias_y = this->gyro_bias_y * this->correction_speed + (this->gyroscope_y + gyro_bias_y) * (1.0 - this->correction_speed);
            this->gyro_bias_z = this->gyro_bias_z * this->correction_speed + (this->gyroscope_z + gyro_bias_z) * (1.0 - this->correction_speed);
            // Knock the timer back a little so we don't update too frequently.
            this->auto_calib_timer *= 0.8;
            return true;
        }
};

/** \brief Compute the mean value of a 1-dimensional ArrayType of doubles.
 * \param[in] data An iterable ArrayType of 1 dimension containing doubles.
 * \return The mean of the double values in the data array.
 */
template <typename ArrayType>
static double array_mean_1d(const ArrayType& data)
{
    double total = 0;
    for (const double& element : data)
    {
        total += element;
    }
    return total / data.size();
}

#endif
