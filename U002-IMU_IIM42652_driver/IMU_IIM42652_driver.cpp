#include <thread>
#include <chrono>

#include "IMU_IIM42652_driver.hpp"

IMU_IIM42652::IMU_IIM42652(int bus_address, std::string bus_name)
{
    // Default configuration.
    unsigned char accel_config = ACCEL_ODR_200HZ | ACCEL_FSR_16G;
    unsigned char gyro_config = GYRO_ODR_200HZ | GYRO_FSR_2000;

    i2c_init(bus_address, bus_name);
    soft_reset();
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
    IMU_config(gyro_config, accel_config);
    set_resolutions(ACCEL_RES_16G, GYRO_RES_2000);
    enable();
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
}

//sets up the i2c handler and connect to a device on the i2c bus
void IMU_IIM42652::i2c_init(const int address, std::string bus_name)
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

//reads from a given register of the IMU
int IMU_IIM42652::i2c_read(int reg_addr, int bytes, uint8_t *buffer)
{
    return i2c_smbus_read_i2c_block_data(this->handle, reg_addr, bytes, buffer);
}

//writes data to a given register on the IMU
int IMU_IIM42652::i2c_write(int reg_addr, int bytes, uint8_t *buffer)
{
    return i2c_smbus_write_i2c_block_data(this->handle, reg_addr, bytes, buffer);
}

//reads the registers containing acclerometer and gyroscope data - doesnt convert data into usable units
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
    accelerometer_x = val;

    val = buffer[3] | (buffer[2] << 8);
    accelerometer_y = val;

    val = buffer[5] | (buffer[4] << 8);
    accelerometer_z = val;

    val = buffer[7] | (buffer[6] << 8);
    gyroscope_x = val;

    val = buffer[9] | (buffer[8] << 8);
    gyroscope_y = val;

    val = buffer[11] | (buffer[10] << 8);
    gyroscope_z = val;
}

//writes to the config registers with given data
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
    this->temperature = (temp / 132.48) + 25;   // Conversion factor from datasheet.
    
    return this->temperature;
}

//reads the registers containing acclerometer and gyroscope data then converts the data into usable units
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

//sets the variables holding resolution data with given data
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