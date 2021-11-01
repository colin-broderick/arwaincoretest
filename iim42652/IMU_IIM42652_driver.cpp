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

/** \brief Produces a random number from a uniform distribution [-1, 1]. */
static double rn()
{
    static std::default_random_engine gen;
    static std::uniform_real_distribution dist{-1.0, 1.0};
    return dist(gen);
}

/** \brief Constructor.
 * \param[in] bus_address The I2C address of the device.
 * \param[in] bus_name The bus on which the device is found, e.g. "/dev/i2c-1".
 */
IMU_IIM42652::IMU_IIM42652(int bus_address, const std::string &bus_name)
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

/** \brief This calibration approach is based on acquistion of accelerometer data at random static orientations.
 * An optimisation step attempts to compute the best combination of misalignment, scale, and bias corrections
 * to fit the provided data and given magnitude of acceleration due to gravity, 9.8607 m/s2.
 */
vector3 IMU_IIM42652::calibrate_accelerometer()
{
    // Collect samples =============================================================
    std::vector<Eigen::Matrix<double, 3, 1>> samples;

    for (int i = 0; i < 12; i++)
    {
        std::cout << i+1 << ") Place the IMU in a random orientation ..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds{5});

        kalman_filter_constant_1d kfx{9.81, 1.0, 0.000001};
        kalman_filter_constant_1d kfy{9.81, 1.0, 0.000001};
        kalman_filter_constant_1d kfz{9.81, 1.0, 0.000001};
        
        while (!kfx.converged && !kfy.converged && !kfz.converged)
        {
            this->read_IMU();
            kfx.update(this->accelerometer_x, 0.1);
            kfy.update(this->accelerometer_y, 0.1);
            kfz.update(this->accelerometer_z, 0.1);
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
        }

        samples.push_back({kfx.est, kfy.est, kfz.est});
        // TODO Add delay and instruction to reorient.
    }    

    // Initial parameter estimates assuming perfect measurements.
    Eigen::Matrix<double, 3, 3> alignment_correction{
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    Eigen::Matrix<double, 3, 3> scale_correction{
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    Eigen::Matrix<double, 3, 1> bias_correction{
        {0}, {0}, {0}
    };

    double alignment_variance = 0.1;
    double scale_variance = 0.1;
    double bias_variance = 0.1;

    // Generate random correction candidates.
    std::vector<AccelerometerCalibrationCandidate> candidates;
    for (int i = 0; i < 1000; i++)
    {
        AccelerometerCalibrationCandidate candidate;
        candidate.alignment = alignment_correction + alignment_variance * Eigen::Matrix<double, 3, 3>{{0, rn(), rn()}, {0, 0, rn()}, {0, 0, 0}};
        candidate.scale = scale_correction + scale_variance * Eigen::Matrix<double, 3, 3>{{rn(), 0, 0}, {0, rn(), 0}, {0, 0, rn()}};
        candidate.bias = bias_correction + bias_variance * Eigen::Matrix<double, 3, 1>{{rn()}, {rn()}, {rn()}};
        candidate.update_loss(9.81, samples);
        candidates.push_back(candidate);
    }

    // Sort the candidates from best to worst.
    std::sort(candidates.begin(), candidates.end(), [](AccelerometerCalibrationCandidate a, AccelerometerCalibrationCandidate b){ return a.loss < b.loss; });

    // Drop the worst 90% of candidates and reduce future variances.
    candidates.resize(100);
    alignment_variance *= 0.1;
    scale_variance *= 0.1;
    bias_variance *= 0.1;

    // For n iterations
    for (int i = 0; i < 100; i++)
    {
        // Generate 9 new candidates by mutating each remaining candidate.
        for (int j = 0; j < 9; j++)
        {
            AccelerometerCalibrationCandidate candidate;
            candidate.alignment = candidates[j].alignment + alignment_variance * Eigen::Matrix<double, 3, 3>{{0, rn(), rn()}, {0, 0, rn()}, {0, 0, 0}};
            candidate.scale = candidates[j].scale + scale_variance * Eigen::Matrix<double, 3, 3>{{rn(), 0, 0}, {0, rn(), 0}, {0, 0, rn()}};
            candidate.bias = candidates[j].bias + bias_variance * Eigen::Matrix<double, 3, 1>{{rn()}, {rn()}, {rn()}};
            candidate.update_loss(9.81, samples);
            candidates.push_back(candidate);
        }

        // Sort the candidates.
        std::sort(candidates.begin(), candidates.end(), [](AccelerometerCalibrationCandidate a, AccelerometerCalibrationCandidate b){ return a.loss < b.loss; });

        // Prune the weakest 90%.
        candidates.resize(100);
    }

    return {candidates[0].bias(0, 0), candidates[0].bias(1, 0), candidates[0].bias(2, 0)};
}

/** \brief Defines a procedure for calculating calibration offsets
 * for the gyroscope, then records those values in the appropriate
 * registers on the device.
 * 
 * We leave the device undistrubed for a set period of time, and
 * then compute the average value from each gyroscope axis during
 * that time. This number is zero in the ideal case. The actual
 * calculated value is the offset that should be applied to the
 * gyroscope readings.
 * 
 * This procedure does not consider the change in offset as a
 * function of temperature.
 */
vector3 IMU_IIM42652::calibrate_gyroscope()
{
    kalman_filter_constant_1d kfx{0, 0.5, 0.0000051};
    kalman_filter_constant_1d kfy{0, 0.5, 0.0000051};
    kalman_filter_constant_1d kfz{0, 0.5, 0.0000051};
    while (!kfx.converged && !kfy.converged && !kfz.converged)
    {
        this->read_IMU();
        kfx.update(this->gyroscope_x, 0.02);
        kfy.update(this->gyroscope_y, 0.02);
        kfz.update(this->gyroscope_z, 0.02);
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    return {kfx.est, kfy.est, kfz.est};
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

/** \brief Fetches raw IMU data and applies conversion factors to get correct units. Alters state. */
void IMU_IIM42652::read_IMU()
{
    read_IMU_raw_data();

    //accelerometer values
    accelerometer_x = accelerometer_x * accel_resolution * GRAVITY;
    accelerometer_y = accelerometer_y * accel_resolution * GRAVITY;
    accelerometer_z = accelerometer_z * accel_resolution * GRAVITY;

    //gyroscope values
    gyroscope_x = gyroscope_x * gyro_resolution * PI_DIVIDED_BY_180;
    gyroscope_y = gyroscope_y * gyro_resolution * PI_DIVIDED_BY_180;
    gyroscope_z = gyroscope_z * gyro_resolution * PI_DIVIDED_BY_180;
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

// Sample main function
/*
int main()
{
    IMU_IIM42652 IMU1(0x68, "/dev/i2c-1");
    IMU_IIM42652 IMU2(0x69, "/dev/i2c-1");
    IMU_IIM42652 IMU3(0x68, "/dev/i2c-4");

    double X = 0;
    double Y = 0;
    double Z = 0;

    while (true)
    {
        std::chrono::milliseconds interval{100};
        IMU1.read_IMU();
        IMU2.read_IMU();
        IMU3.read_IMU();

        X = (IMU1.accelerometer_x + IMU2.accelerometer_x + IMU3.accelerometer_x) / 3;
        Y = (IMU1.accelerometer_y + IMU2.accelerometer_y + IMU3.accelerometer_y) / 3;
        Z = (IMU1.accelerometer_z + IMU2.accelerometer_z + IMU3.accelerometer_z) / 3;

        std::cout << "x: " << X << "\t"
                  << "y: " << Y << "\t"
                  << "z: " << Z << "\n";

        std::this_thread::sleep_for(interval);
    }
}
*/