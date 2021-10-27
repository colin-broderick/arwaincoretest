#include <thread>
#include <chrono>
#include <array>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <stdlib.h>
#include <algorithm>

#include "IMU_IIM42652_driver.hpp"

#define V2_CALIB

/** \brief Generate a random number between -size and size, centred on zero. */
static double rn(double size)
{
    return ((double)RAND_MAX / (double)rand() - 0.5) * 2 * size;
}

/** \brief Constructor.
 * \param[in] bus_address The I2C address of the device.
 * \param[in] bus_name The bus on which the device is found, e.g. "/dev/i2c-1".
 */
IMU_IIM42652::IMU_IIM42652(int bus_address, const std::string &bus_name)
{
    // Default configuration.
    unsigned char accel_config = ACCEL_200HZ | ACCEL_FSR_16G;
    unsigned char gyro_config = GYRO_200HZ | GYRO_FSR_2000;

    i2c_init(bus_address, bus_name);
    soft_reset();
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
    IMU_config(gyro_config, accel_config);
    set_resolutions(ACCEL_RES_16G, GYRO_RES_2000);
    enable();
    std::this_thread::sleep_for(std::chrono::milliseconds{25});
}

#ifdef V1_CALIB
/** \brief Defines a procedure for calculating calibration offsets
 * for the accelerometer, then records those values in the appropriate
 * registers on the device.
 * 
 * We position the device with the x axis pointing up, and log ten seconds
 * worth of data. We then position the device with the x axis pointing down
 * and log ten more seconds of data. All of this data goes into the same array.
 * The mean value of this array should be zero. The actual mean is the offset
 * that should be applied to accelerometer readings on the x axis.
 * 
 * Repeat this procedure for the y and z axis.
 */
void IMU_IIM42652::calibrate_accelerometer()
{
    // Set up timing and create data storage.
    const int frequency = 100;
    const int logging_seconds = 2;
    const int maneuvering_seconds = 1;
    const std::chrono::milliseconds reading_interval{1000 / frequency};
    const std::chrono::seconds maneuvering_time{maneuvering_seconds};
    const std::chrono::seconds logging_time{logging_seconds};
    const int num_samples_per_round = frequency * logging_seconds;

    uint8_t bf;
    i2c_read(0x7B, 1, &bf);
    std::cout << (int)bf << std::endl;

    std::array<double, num_samples_per_round * 2> x_data;
    std::array<double, num_samples_per_round * 2> y_data;
    std::array<double, num_samples_per_round * 2> z_data;

    // Log x-up data.
    std::cout << "Place the device with the accelerometer x axis pointing up" << std::endl;
    std::cout << "Starting calibration log in " << maneuvering_time.count() << " seconds; ensure the device is undisturbed" << std::endl;
    std::this_thread::sleep_for(maneuvering_time);
    std::cout << "Logging data for " << logging_time.count() << " seconds; do not move or vibrate the device" << std::endl;

    for (int i = 0; i < num_samples_per_round; i++)
    {
        this->read_IMU();
        x_data[i] = this->accelerometer_x;
        std::this_thread::sleep_for(reading_interval);
    }

    // Log x-down data.
    std::cout << "Place the device with the accelerometer x axis pointing down" << std::endl;
    std::cout << "Starting calibration log in " << maneuvering_time.count() << " seconds; ensure the device is undisturbed" << std::endl;
    std::this_thread::sleep_for(maneuvering_time);
    std::cout << "Logging data for " << logging_time.count() << " seconds; do not move or vibrate the device" << std::endl;

    for (int i = 0; i < num_samples_per_round; i++)
    {
        this->read_IMU();
        x_data[i + num_samples_per_round] = this->accelerometer_x;
        std::this_thread::sleep_for(reading_interval);
    }

    // Log y-up data.
    std::cout << "Place the device with the accelerometer y axis pointing up" << std::endl;
    std::cout << "Starting calibration log in " << maneuvering_time.count() << " seconds; ensure the device is undisturbed" << std::endl;
    std::this_thread::sleep_for(maneuvering_time);
    std::cout << "Logging data for " << logging_time.count() << " seconds; do not move or vibrate the device" << std::endl;

    for (int i = 0; i < num_samples_per_round; i++)
    {
        this->read_IMU();
        y_data[i] = this->accelerometer_y;
        std::this_thread::sleep_for(reading_interval);
    }

    // Log y-down data.
    std::cout << "Place the device with the accelerometer y axis pointing down" << std::endl;
    std::cout << "Starting calibration log in " << maneuvering_time.count() << " seconds; ensure the device is undisturbed" << std::endl;
    std::this_thread::sleep_for(maneuvering_time);
    std::cout << "Logging data for " << logging_time.count() << " seconds; do not move or vibrate the device" << std::endl;

    for (int i = 0; i < num_samples_per_round; i++)
    {
        this->read_IMU();
        y_data[i + num_samples_per_round] = this->accelerometer_y;
        std::this_thread::sleep_for(reading_interval);
    }

    // Log z-up data.
    std::cout << "Place the device with the accelerometer z axis pointing up" << std::endl;
    std::cout << "Starting calibration log in " << maneuvering_time.count() << " seconds; ensure the device is undisturbed" << std::endl;
    std::this_thread::sleep_for(maneuvering_time);
    std::cout << "Logging data for " << logging_time.count() << " seconds; do not move or vibrate the device" << std::endl;

    for (int i = 0; i < num_samples_per_round; i++)
    {
        this->read_IMU();
        z_data[i] = this->accelerometer_z;
        std::this_thread::sleep_for(reading_interval);
    }

    // Log z-down data.
    std::cout << "Place the device with the accelerometer z axis pointing down" << std::endl;
    std::cout << "Starting calibration log in " << maneuvering_time.count() << " seconds; ensure the device is undisturbed" << std::endl;
    std::this_thread::sleep_for(maneuvering_time);
    std::cout << "Logging data for " << logging_time.count() << " seconds; do not move or vibrate the device" << std::endl;

    for (int i = 0; i < num_samples_per_round; i++)
    {
        this->read_IMU();
        z_data[i + num_samples_per_round] = this->accelerometer_z;
        std::this_thread::sleep_for(reading_interval);
    }

    // Compute the accelerometer offsets based on the log data.
    double x_offset = array_mean_1d(x_data);
    double y_offset = array_mean_1d(y_data);
    double z_offset = array_mean_1d(z_data);

    // Set the appropriate register values
    std::cout << "ax: " << x_offset << std::endl;
    std::cout << "ay: " << y_offset << std::endl;
    std::cout << "az: " << z_offset << std::endl;
}
#else
/** \brief This calibration approach is based on acquistion of accelerometer data at random static orientations.
 * An optimisation step attempts to compute the best combination of misalignment, scale, and bias corrections
 * to fit the provided data and given magnitude of acceleration due to gravity, 9.8607 m/s2.
 */
void IMU_IIM42652::calibrate_accelerometer()
{
    IMU_IIM42652 imu{0x68, "/dev/i2c-1"};
    std::vector<Eigen::Matrix<double, 3, 1>> samples;

    double gravity = 9.8607;

    double alignment_variance = 0.1;
    double a_yz = 0, a_zy = 0, a_zx = 0; // Axis alignment correction factors.

    double scale_variance = 0.1;
    double s_ax = 1, s_ay = 1, s_az = 1; // Scale correction factors.

    double bias_variance = 0.1;
    double b_ax = 0, b_ay = 0, b_az = 0; // Bias correction offsets.

    std::cout << "Randomly orient the sensor. First of 20 sample sets will be logged in ten seconds ..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds{10});

    // Record accelerometer samples.
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 20; j++)
        {
            imu.read_IMU();
            samples.emplace_back(Eigen::Matrix<double, 3, 1>{
                {imu.accelerometer_x},
                {imu.accelerometer_y},
                {imu.accelerometer_z}});
            std::this_thread::sleep_for(std::chrono::milliseconds{10});
        }
        std::cout << "Sample set " << i + 1 << " logged. Randomly reorient the sensor within five seconds ..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds{2});
    }

    // Axis misalignment matrix.
    Eigen::Matrix<double, 3, 3> alignment{
        {1, -a_yz, a_zy},
        {0, 1, -a_zx},
        {0, 0, 1}};
    Eigen::Matrix<double, 3, 3> alignment_ones{
        {0, 1, 1},
        {0, 0, 1},
        {0, 0, 0}};

    // Scale factor matrix.
    Eigen::Matrix<double, 3, 3> scale{
        {s_ax, 0, 0},
        {0, s_ay, 0},
        {0, 0, s_az}};
    Eigen::Matrix<double, 3, 3> scale_ones{
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}};

    // Bias matrix.
    Eigen::Matrix<double, 3, 1> bias{
        {b_ax},
        {b_ay},
        {b_az}};
    Eigen::Matrix<double, 3, 1> bias_ones{
        {1},
        {1},
        {1}};

    // Compute the initial loss
    double loss = 0;
    for (Eigen::Matrix<double, 3, 1> &sample : samples)
    {
        auto h = alignment * scale * (sample - bias);
        loss = loss + (gravity * gravity - h.dot(h)) * (gravity * gravity - h.dot(h));
    }

    // Perform iterative optimisation steps, modifying the correcting factors to minimise loss.

    struct Candidate
    {
        Eigen::Matrix<double, 3, 3> alignment;
        Eigen::Matrix<double, 3, 3> scale;
        Eigen::Matrix<double, 3, 1> bias;
        double loss = 0;
        void update_loss(double gravity, std::vector<Eigen::Matrix<double, 3, 1>>& samples)
        {
            this->loss = 0;
            for (Eigen::Matrix<double, 3, 1> &sample : samples)
            {
                auto h = this->alignment * this->scale * (sample - this->bias);
                auto hdoth = h.dot(h);
                auto x = gravity * gravity - hdoth;
                this->loss = this->loss + x * x;
                // std::cout << "hdoth: " << hdoth << std::endl;
                // std::cout << "x:     " << x << std::endl;
                // std::cout << "loss:  " << this->loss << std::endl;
            }
        }
    };

    std::vector<Candidate> candidates;
    for (int i = 0; i < 1000; i++)
    {
        // Generate first generation of guesses
        Candidate cnd;
        cnd.alignment = alignment + alignment_variance * Eigen::Matrix<double, 3, 3>{{0, rn(1), rn(1)}, {0, 0, rn(1)}, {0, 0, 0}};
        cnd.scale = scale + scale_variance * Eigen::Matrix<double, 3, 3>{{rn(1), 0, 0}, {0, rn(1), 0}, {0, 0, rn(1)}};
        cnd.bias = bias + bias_variance * Eigen::Matrix<double, 3, 1>{{rn(1)}, {rn(1)}, {rn(1)}};
        candidates.push_back(cnd);
    }

    // Compute the loss for each candidiate and sort. Reduce to best 10%.
    for (Candidate &candidate : candidates)
    {
        candidate.update_loss(gravity, samples);
    }
    std::sort(candidates.begin(), candidates.end(), [](Candidate a, Candidate b){ return a.loss < b.loss; });
    candidates.resize(100);

    // Reduce perturbation size and iterate until convergence.
    alignment_variance *= 0.1;
    scale_variance *= 0.1;
    bias_variance *= 0.1;

    // Regenerate the population and repeatedly prune weaker candidates.
    for (int i = 0; i < 100; i++)
    {
        // Generate 9 new candidates from each remaining candidate.
        for (int j = 0; j < 100; j++)
        {
            for (int k = 0; k < 9; k++)
            {
                Candidate new_candidate;
                new_candidate.alignment = candidates[j].alignment + alignment_variance * Eigen::Matrix<double, 3, 3>{{0, rn(1), rn(1)}, {0, 0, rn(1)}, {0, 0, 0}};
                new_candidate.scale = candidates[j].scale + scale_variance * Eigen::Matrix<double, 3, 3>{{rn(1), 0, 0}, {0, rn(1), 0}, {0, 0, rn(1)}};
                new_candidate.bias = candidates[j].bias + bias_variance * Eigen::Matrix<double, 3, 1>{{rn(1)}, {rn(1)}, {rn(1)}};
                candidates.push_back(new_candidate);
            }
        }

        for (Candidate& candidate : candidates)
        {
            candidate.update_loss(gravity, samples);
        }

        // Sort all candidates.
        std::sort(candidates.begin(), candidates.end(), [](Candidate a, Candidate b){ return a.loss < b.loss; });

        // Prune weakest 90%.
        candidates.resize(100);

        std::cout << candidates[0].loss << ", " << candidates[99].loss << std::endl;
    }

    // Display or store the final values.
    std::cout << "algt: " << candidates[0].alignment << std::endl;
    std::cout << "scle: " << candidates[0].scale << std::endl;
    std::cout << "bias: " << candidates[0].bias << std::endl;
}
#endif

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
void IMU_IIM42652::calibrate_gyroscope()
{
    const int num_samples = 1000;
    std::array<std::array<double, num_samples>, 3> data;
    std::chrono::milliseconds reading_interval{10};
    std::cout << "Gyroscope calibration will begin in 5 seconds; do not move the device." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds{5});
    std::cout << "Calibration has started. Do not move the device for 10 seconds." << std::endl;

    for (int i = 0; i < num_samples; i++)
    {
        this->read_IMU();
        data[0][i] = this->gyroscope_x;
        data[1][i] = this->gyroscope_y;
        data[2][i] = this->gyroscope_z;
        std::this_thread::sleep_for(reading_interval);
    }
    double gx = array_mean_1d(data[0]);
    double gy = array_mean_1d(data[1]);
    double gz = array_mean_1d(data[2]);

    // Store offsets in appropriate registers.
    std::cout << "gx: " << gx << std::endl;
    std::cout << "gy: " << gy << std::endl;
    std::cout << "gz: " << gz << std::endl;
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