#include <thread>
#include <chrono>

#include "mlx90395.hpp"

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
    uint8_t data = 0x80;
    i2c_write(CMD_TARGET, 1, &data);
}

/** \brief Read magnetometer data. */
vector3 MLX90395::read()
{
    // TODO
    // TEST send read command
    send_read_command();

    // reader 12 bytes
    uint8_t buffer[12];
    i2c_read(READ_TARGET, 12, buffer);

    // TODO check buf[0] contains data rdy flag

    // convert buffer into doubles
    // TEST Check if these can be doubles instead.
    mag_x = (buffer[2] << 8) | buffer[3];
    mag_y = (buffer[4] << 8) | buffer[5];
    mag_z = (buffer[6] << 8) | buffer[7];

    mag_x = gain_multipliers[mag_gain] * uTLSB;
    mag_y = gain_multipliers[mag_gain] * uTLSB;
    mag_z = gain_multipliers[mag_gain] * uTLSB;

    return {mag_x, mag_y, mag_z};
}

/** \brief Read in current config, then write back with reset bit set. Wait 2 ms after reset. */
bool MLX90395::reset()
{
    // TEST
    uint8_t data = CMD_RESET;
    int ret = i2c_write(CMD_TARGET, 1, &data);
    return ret;
}

/** \brief Set magnetometer to default configuraion. */
bool MLX90395::configure()
{
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

bool MLX90395::exit_mode()
{
    // TEST
    uint8_t data = CMD_EXIT;
    i2c_write(CMD_TARGET, 1, &data);
    // i2c_write(CMD_TARGET, 1, &data); // Might need to be done twice.
    return true;
}

uint8_t MLX90395::get_gain()
{
    // TEST Read two bytes from register 0x00, shift right 4, keep 4
    uint8_t data[2];
    i2c_read(0x00, 2, data);
    return (data[1] >> 4) & 0b00001111;
}

uint8_t MLX90395::get_resolution()
{
    // TEST read two bytes from 0x04, shift right by 5, take 2 smallest bits
    uint8_t data[2];
    i2c_read(0x04, 2, data);
    return (data[1] >> 5) & 0b00000011;
}

void MLX90395::set_osr(uint8_t osr_val)
{
    // TODO
    // Read current register.
    // Or with chosen OSR.
    // Write register.
}

void MLX90395::set_resolution(uint8_t res)
{
    // TODO
    // Read current register.
    // Or with chosen OSR.
    // Write register.
}