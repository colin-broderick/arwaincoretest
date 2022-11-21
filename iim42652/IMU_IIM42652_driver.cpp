#include <thread>
#include <chrono>
#include <array>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <stdlib.h>
#include <algorithm>
#include <random>

#include "IMU_IIM42652_driver.hpp"
#include "kalman.hpp"
#include "vector3.hpp"

/** \brief Constructor.
 * \param[in] bus_address The I2C address of the device.
 * \param[in] bus_name The bus on which the device is found, e.g. "/dev/i2c-1".
 */
IMU_IIM42652::IMU_IIM42652(int bus_address, const std::string &bus_name)
: address_(bus_address), bus_name_(bus_name)
{
    // Default configuration.
    unsigned char accel_config = ACCEL_200HZ | ACCEL_FSR_16G;
    unsigned char gyro_config = GYRO_200HZ | GYRO_FSR_1000;

    i2c_init(bus_address, bus_name);
    soft_reset();
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
    IMU_config(gyro_config, accel_config);
    set_resolutions(ACCEL_RES_16G, GYRO_RES_1000);
    enable();
    std::this_thread::sleep_for(std::chrono::milliseconds{25});
}

Vector3 IMU_IIM42652::get_gyro_calib()
{
    return {gyro_bias_x, gyro_bias_y, gyro_bias_z};
}

double IMU_IIM42652::get_gyro_calib_x()
{
    return gyro_bias_x;
}

double IMU_IIM42652::get_gyro_calib_y()
{
    return gyro_bias_y;
}

double IMU_IIM42652::get_gyro_calib_z()
{
    return gyro_bias_z;
}

namespace
{
    struct AccelerometerCalibrationCandidate
    {
        Eigen::Matrix<double, 3, 3> alignment;
        Eigen::Matrix<double, 3, 3> scale;
        Eigen::Matrix<double, 3, 1> bias;
        double loss = 0;
        AccelerometerCalibrationCandidate()
        {
        }
        void update_loss(double gravity, std::vector<Eigen::Matrix<double, 3, 1>>& samples)
        {
            this->loss = 0;
            for (Eigen::Matrix<double, 3, 1> &sample : samples)
            {
                auto h = this->alignment * this->scale * (sample - this->bias);
                auto hdoth = h.dot(h);
                auto x = gravity * gravity - hdoth;
                this->loss = this->loss + x * x;
            }
        }
    };
}

Vector3 IMU_IIM42652::calibration_accel_sample()
{
    KalmanFilter1D kfx{9.81, 1.0};
    KalmanFilter1D kfy{9.81, 1.0};
    KalmanFilter1D kfz{9.81, 1.0};
    
    while (!kfx.converged && !kfy.converged && !kfz.converged)
    {
        auto [accel, gyro] = this->read_IMU();
        kfx.update(accel.x, 0.1);
        kfy.update(accel.y, 0.1);
        kfz.update(accel.z, 0.1);
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }

    return {kfx.est, kfy.est, kfz.est};
}

/** \brief Defines a procedure for calculating calibration offsets
 * for the gyroscope.
 * 
 * We take repeated gyroscope readings until a state estimator converges
 * on a stable value for each gyroscope axis.
 * 
 * This procedure does not consider the change in offset as a
 * function of temperature.
 */
Vector3 IMU_IIM42652::calibrate_gyroscope()
{
    KalmanFilter1D kfx{0, 1};
    KalmanFilter1D kfy{0, 1};
    KalmanFilter1D kfz{0, 1};
    while (!kfx.converged && !kfy.converged && !kfz.converged)
    {
        auto [accel, gyro] = this->read_IMU();
        kfx.update(gyro.x, 0.02);
        kfy.update(gyro.y, 0.02);
        kfz.update(gyro.z, 0.02);
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    return {kfx.est, kfy.est, kfz.est};
}

int IMU_IIM42652::get_address() const
{
    return address_;
}

std::string IMU_IIM42652::get_bus() const
{
    return bus_name_;
}

/** \brief Sets up the I2C file handle and connects to a device on the I2C bus.
 * \param[in] address The address of the device on the I2C bus.
 * \param[in] bus_name The name of the bus to open, e.g. /dev/i2c-1.
 */
void IMU_IIM42652::i2c_init(const int address, const std::string &bus_name)
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
int IMU_IIM42652::i2c_read(int reg_addr, int bytes, uint8_t *buffer)
{
    return i2c_smbus_read_i2c_block_data(this->handle, reg_addr, bytes, buffer);
}

/** \brief Writes a given number of bytes to a specified register address.
 * \param[in] reg_addr The register address to start writing.
 * \param[in] bytes The number of bytes to write.
 * \param[in] buffer The source of the bytes to be written.
 * \return SMBUS return code.
 */
int IMU_IIM42652::i2c_write(int reg_addr, int bytes, uint8_t *buffer)
{
    return i2c_smbus_write_i2c_block_data(this->handle, reg_addr, bytes, buffer);
}

/** \brief Reads the registers containing raw accelerometer and gyroscope data. */
void IMU_IIM42652::read_IMU_raw_data()
{
    uint8_t buffer[12];
    int ret_code;
    int16_t val = 0;

    ret_code = i2c_read(ADDR_ALL_IMU, 12, buffer);
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

/** \brief Writes FSR and ODR settings to device registers.
 * \param gyro_config Four bytes of gyro ODR | four bytes of gyro FSR.
 * \param gyro_config Four bytes of accel ODR | four bytes of accel FSR.
*/
int IMU_IIM42652::IMU_config(uint8_t gyro_config, uint8_t accel_config)
{
    int ret_code = i2c_write(ADDR_GYRO_CONFIG0, 1, &gyro_config);
    if (ret_code > 0)
    {
        return ret_code;
    }
    ret_code = i2c_write(ADDR_ACCEL_CONFIG0, 1, &accel_config);
    if (ret_code > 0)
    {
        return ret_code;
    }

    // Set internal gyro filter order.
    uint8_t val = GYRO_UI_FILTER_ORD_3RD | 0b00000010;
    ret_code = i2c_write(ADDR_GYRO_CONFIG1, 1, &val);
    if (ret_code > 0)
    {
        return ret_code;
    }

    return ret_code;
}

/** \brief Read the temperature from the IMU, update the state of this
 * object with the new value.
 * \return The temperature of the sensor in degrees centigrade.
 */
double IMU_IIM42652::read_temperature()
{
    uint8_t bf[2];
    i2c_read(ADDR_TMPRTR, 2, bf);

    int16_t temp = 0;
    temp = (bf[0] << 8) | bf[1];
    this->temperature = (temp / 132.48) + 25; // Conversion factor from datasheet.

    return this->temperature;
}

/** \brief Gives the current auto calibration state. 
 * \return bool indiciating whether autocalibration is enabled; true for on, false for off.
 */
bool IMU_IIM42652::auto_calib_enabled() const
{
    return auto_calib_enabled_;
}

/** \brief Fetches raw IMU data and applies conversion factors to get correct units. Alters state. */
Vector6 IMU_IIM42652::read_IMU()
{
    read_IMU_raw_data();

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

/** \brief Sets accel/gyro conversion factors to match the FSR configuration.
 * \param accel Conversion factor from raw accel data to double representation in units of g.
 * \param gyro Conversion factor from raw gyro data to double representation in units of degrees per second.
 */
void IMU_IIM42652::set_resolutions(double accel, double gyro)
{
    accel_resolution = accel;
    gyro_resolution = gyro;
}

/** \brief Read in current config, then write back with reset bit set. Wait 2 ms after reset. */
void IMU_IIM42652::soft_reset()
{
    uint8_t buffer = CONFIG_SOFT_RESET;
    i2c_write(ADDR_DEVICE_CONFIG, 1, &buffer);
}

/** \brief Set register values to enable temperature sensor, accelerometer, gyroscope. */
void IMU_IIM42652::enable()
{
    uint8_t buffer = CONFIG_ENABLE_ACCEL | CONFIG_ENABLE_GYRO | CONFIG_ENABLE_TMPRTR;
    i2c_write(ADDR_PWR_MGMT0, 1, &buffer);
}

/** \brief Set the values of the gyroscope bias offsets, subtracted from the gyro readings before returning.
 */
void IMU_IIM42652::set_gyro_bias(double x, double y, double z)
{
    gyro_bias_x = x;
    gyro_bias_y = y;
    gyro_bias_z = z;
}

void IMU_IIM42652::set_accel_bias(double x, double y, double z)
{
    accel_bias_x = x;
    accel_bias_y = y;
    accel_bias_z = z;
}

void IMU_IIM42652::set_accel_scale(double x, double y, double z)
{
    accel_scale_x = x;
    accel_scale_y = y;
    accel_scale_z = z;
}

/** \brief Automatically update the gyroscope bias when the device is detected to be stationary.
 */
void IMU_IIM42652::update_gyro_bias()
{
    // If any gyro readings are above the auto calib threshold, reset the timer.
    if (std::abs(this->gyroscope_x) > auto_calib_threshold || std::abs(this->gyroscope_y) > auto_calib_threshold || std::abs(this->gyroscope_z) > this->auto_calib_threshold)
    {
        this->auto_calib_timer = 0;
        return;
    }

    // If the minimum static time has not yet elapsed, increment the timer.
    if (this->auto_calib_timer < this->calib_time)
    {
        this->auto_calib_timer++;
        return;
    }

    // If not gyro reading breaks the threshold, and the timer has elapsed, use the new reading to update the bias.
    // And knock the timer back a bit.
    this->gyro_bias_x = this->gyro_bias_x * this->correction_speed + (this->gyroscope_x + gyro_bias_x) * (1.0 - this->correction_speed);
    this->gyro_bias_y = this->gyro_bias_y * this->correction_speed + (this->gyroscope_y + gyro_bias_y) * (1.0 - this->correction_speed);
    this->gyro_bias_z = this->gyro_bias_z * this->correction_speed + (this->gyroscope_z + gyro_bias_z) * (1.0 - this->correction_speed);
    // Knock the timer back a little so we don't update too frequently.
    this->auto_calib_timer *= 0.8;
}

/** \brief Set the gain on the EWMA filter which adjusts gyro bias.
 * \param speed Value between 0 and 1; update step is prev * speed + new * (1 - speed).
 */
void IMU_IIM42652::set_correction_speed(double speed)
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

void IMU_IIM42652::enable_auto_calib()
{
    auto_calib_enabled_ = true;
}

void IMU_IIM42652::enable_auto_calib(double threshold)
{
    this->auto_calib_threshold = threshold;
    this->enable_auto_calib();
}

void IMU_IIM42652::disable_auto_calib()
{
    auto_calib_enabled_ = false;
}
