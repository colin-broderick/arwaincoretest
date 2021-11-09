#include <thread>
#include <chrono>

#include "mlx90395.hpp"

static const std::array<float, 16> gain_multipliers = {
        0.2, 0.25, 0.3333, 0.4, 0.5, 0.6, 0.75, 1,
        0.1, 0.125, 0.1667, 0.2, 0.25, 0.3, 0.375, 0.5};

/** \brief Constructor.
 * \param[in] bus_address The I2C address of the device.
 * \param[in] bus_name The bus on which the device is found, e.g. "/dev/i2c-1".
 */
MLX90395::MLX90395(int bus_address, const std::string &bus_name)
{
    i2c_init(bus_address, bus_name);
    configure();
    std::this_thread::sleep_for(std::chrono::milliseconds{2});
}

/** \brief Measure the current magnetic field vector and compute the rotation required
 * to rotate that vector onto the expected local magnetic field.
 * \return A versor which applies the specified rotation.
 */
quaternion MLX90395::read_orientation()
{
    // static vector3 mag_target{18.895, -0.361, 45.372}; // The local magnetic field vector.
    static vector3 mag_target{0.38443, -0.0073448, 0.92312}; // The normalized local magnetic field vector.
    
    vector3 mag_measurement = this->read().normalized(); // Normalized magnetic field vector.

    double angle = std::acos(mag_measurement.x * mag_target.x 
                           + mag_measurement.y * mag_target.y
                           + mag_measurement.z * mag_target.z); // The angle between the measured field and the local field.

    vector3 axis = vector3::cross(mag_measurement, mag_target); // Axis orthogonal to both measured and expected.

    quaternion quat{
        std::cos(angle/2.0),
        std::sin(angle/2.0) * axis.x,
        std::sin(angle/2.0) * axis.y,
        std::sin(angle/2.0) * axis.z
    }; // Rotation operator to rotate measured vector onto expected vector.

    return quat;
}

/** \brief Rate can be between 1 and 50 (ish). 
 * The time between reads will be floor(1000 / rate) ms.
*/
bool MLX90395::set_data_rate(uint8_t rate)
{
    if (rate < 1 || rate > 50)
    {
        return false;
    }
 
    int ms_per_step = 20;
    
    uint8_t step = 1000 / rate / ms_per_step;
    
    uint8_t bf[2];
    i2c_read(REGISTER_1, 2, bf);
    bf[1] |= step;
    i2c_write(REGISTER_1, 2, bf);
    
    return true;
}

/** \brief Sets up the I2C file handle and connects to a device on the I2C bus.
 * \param[in] address The address of the device on the I2C bus.
 * \param[in] bus_name The name of the bus to open, e.g. /dev/i2c-1.
 */
void MLX90395::i2c_init(const int address, const std::string &bus_name)
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
int MLX90395::i2c_read(int reg_addr, int bytes, uint8_t *buffer)
{
    return i2c_smbus_read_i2c_block_data(this->handle, reg_addr, bytes, buffer);
}

/** \brief Writes a given number of bytes to a specified register address.
 * \param[in] reg_addr The register address to start writing.
 * \param[in] bytes The number of bytes to write.
 * \param[in] buffer The source of the bytes to be written.
 * \return SMBUS return code.
 */
int MLX90395::i2c_write(int reg_addr, int bytes, uint8_t *buffer)
{
    return i2c_smbus_write_i2c_block_data(this->handle, reg_addr, bytes, buffer);
}

void MLX90395::send_read_command()
{
    // TODO Is this really right? 0x80 is also the exit command ...
    uint8_t data = CMD_READ;
    i2c_write(CMD_TARGET, 1, &data);
}

uint8_t MLX90395::command(uint8_t cmd)
{
    // TEST
    uint8_t status;
    i2c_write(CMD_TARGET, 1, &cmd);
    i2c_read(CMD_TARGET, 1, &status);
    return status;
}

/** \brief Read magnetometer data. */
vector3 MLX90395::read()
{
    // TEST

    // TEST send read command
    send_read_command();

    // reader 12 bytes
    uint8_t buffer[12];
    i2c_read(READ_TARGET, 12, buffer);

    // TODO check buf[0] contains data rdy flag

    // convert buffer into doubles
    mag_x = (buffer[2] << 8) | buffer[3];
    mag_y = (buffer[4] << 8) | buffer[5];
    mag_z = (buffer[6] << 8) | buffer[7];

    mag_x = gain_multipliers[mag_gain] * uTLSB;
    mag_y = gain_multipliers[mag_gain] * uTLSB;
    mag_z = gain_multipliers[mag_gain] * uTLSB;

    return {mag_x, mag_y, mag_z};
}

bool MLX90395::reset()
{
    // TEST
    return command(CMD_RESET) == STATUS_RESET;
}

/** \brief Set magnetometer to default configuraion. */
bool MLX90395::configure()
{
    // TEST 
    if (!exit_mode())
    {
        return false;
    }

    if (!reset())
    {
        return false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds{10});

    mag_gain = get_gain();
    if (mag_gain == 8)
    {
        uTLSB = 7.14;
    }
    else
    {
        uTLSB = 2.5;
    }

    mag_resolution = get_resolution();

    return true;
}

uint8_t MLX90395::burst_mode()
{
    // TEST
    return command(CMD_BURST);
}

bool MLX90395::exit_mode()
{
    // TEST
    command(CMD_EXIT);
    return command(CMD_EXIT) == 0;
}

uint8_t MLX90395::get_gain()
{
    // TEST
    uint8_t data[2];
    i2c_read(REGISTER_0, 2, data);
    return (data[1] >> 4) & 0x0F;
}

uint8_t MLX90395::get_resolution()
{
    // TEST
    uint8_t data[2];
    i2c_read(REGISTER_2, 2, data);
    return (data[1] >> 5) & 0x03;
}

/** \brief Gain occupies bits 7:4 of register 0. */
void MLX90395::set_gain(uint8_t gain_)
{
    // TEST
    uint8_t bf[2];
    i2c_read(REGISTER_0, 2, bf);
    bf[1] |= (gain_ << 4);
    i2c_write(REGISTER_0, 2, bf);
}

/** \brief OSR occupies bits 1:0 of register 2. */
void MLX90395::set_osr(OSR osr_val)
{
    // TEST
    uint8_t bf[2];
    i2c_read(REGISTER_2, 2, bf);
    bf[1] |= osr_val;
    i2c_write(REGISTER_2, 2, bf);
}

/** \brief Resolution occupies bits 10:5 of register 2. */
void MLX90395::set_resolution(Resolution res)
{
    // TEST
    uint8_t bf[2];
    i2c_read(REGISTER_2, 2, bf);

    // ResX is in bits 6:5
    bf[1] |= (res << 5);

    // ResY is in bits 8:7
    bf[1] |= (res << 7);
    bf[0] |= (res >> 1);

    // ResZ is in bits 10:9
    bf[0] |= (res << 1);
}
