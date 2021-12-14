#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>

#include "bmp384.hpp"

static void sleep_ms(int ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds{ms});
}

BMP384::BMP384(int bus_address, const std::string &bus_name)
{
    i2c_init(bus_address, bus_name);

    soft_reset();

    get_calib_data();

    set_pressure_osr(PRESSURE_OSR_X1);
    // set_temperature_osr(TEMP_OSR_X2);
    enable_pressure();
    enable_temperature();
    set_power_mode(BMP384_MODE_NORMAL);
    // set_iir_filter(IIR_COEFF_7);
    sleep_ms(10);
}

void BMP384::soft_reset()
{
    uint8_t reset = BMP384_SOFT_RESET;
    i2c_write(ADDR_CMD, 1, &reset);
    sleep_ms(2);
}

std::tuple<double, double> BMP384::read()
{
    uint8_t bf[6];
    i2c_read(ADDR_DATA_0, 6, bf);

    uint32_t pressure = ((uint32_t)(bf[2]) << 16) | ((uint32_t)(bf[1]) << 8) | ((uint32_t)(bf[0]) >> 0);
    uint32_t temperature = ((uint32_t)(bf[5]) << 16) | ((uint32_t)(bf[4]) << 8) | ((uint32_t)(bf[3]) >> 0);

    double temperature_compensated = compensate_temperature(temperature);
    double pressure_compensated = compensate_pressure(pressure);

    return {pressure_compensated, temperature_compensated};
}

double BMP384::compensate_temperature(uint32_t temperature_uncompensated)
{
    double partial_data1 = (double)(temperature_uncompensated - calib_data.par_t1);
    double partial_data2 = (double)(partial_data1 * calib_data.par_t2);

    calib_data.t_lin = partial_data2 + (partial_data1 * partial_data1) * calib_data.par_t3;

    if (calib_data.t_lin < BMP384::min_temp)
    {
        calib_data.t_lin = (double)(BMP384::min_temp);
    }
    if (calib_data.t_lin > BMP384::max_temp)
    {
        calib_data.t_lin = (double)(BMP384::max_temp);
    }

    return calib_data.t_lin;
}

/** \brief Converts the raw temperature reading to something useful.
 * This is not particularly well motivated; the code comes directly from the BMP384 datasheet.
 */
double BMP384::compensate_pressure(uint32_t pressure_uncompensated)
{
    double compensated_pressure, partial_data1, partial_data2, partial_data3, partial_data4, partial_out1, partial_out2;

    partial_data1 = calib_data.par_p6 * calib_data.t_lin;
    partial_data2 = calib_data.par_p7 * calib_data.t_lin * calib_data.t_lin;
    partial_data3 = calib_data.par_p8 * calib_data.t_lin * calib_data.t_lin * calib_data.t_lin;
    partial_out1 = calib_data.par_p5 + partial_data1 + partial_data2 + partial_data3;

    partial_data1 = calib_data.par_p2 * calib_data.t_lin;
    partial_data2 = calib_data.par_p3 * calib_data.t_lin * calib_data.t_lin;
    partial_data3 = calib_data.par_p4 * calib_data.t_lin * calib_data.t_lin * calib_data.t_lin;
    partial_out2 = (double)pressure_uncompensated * (calib_data.par_p1 + partial_data1 + partial_data2 + partial_data3);

    partial_data1 = (double)pressure_uncompensated * (double)pressure_uncompensated;
    partial_data2 = calib_data.par_p9 + calib_data.par_p10 * calib_data.t_lin;
    partial_data3 = partial_data1 * partial_data2;
    partial_data4 = partial_data3 + (double)pressure_uncompensated * (double)pressure_uncompensated * (double)pressure_uncompensated * calib_data.par_p11;

    compensated_pressure = partial_out1 + partial_out2 + partial_data4;

    return compensated_pressure;
}

void BMP384::get_calib_data()
{
    uint8_t bf[21];
    i2c_read(ADDR_CALIB_DATA, 21, bf);
    this->calib_data.par_t1 = (double)(((uint16_t)(bf[1]) << 8) | (uint16_t)(bf[0])) / std::pow(2, -8);
    this->calib_data.par_t2 = (double)(((uint16_t)(bf[3]) << 8) | (uint16_t)(bf[2])) / std::pow(2, 30);
    this->calib_data.par_t3 = (double)((int8_t)(bf[4])) / std::pow(2, 48);
    this->calib_data.par_p1 = ((double)((int16_t)(bf[6] << 8) | (int16_t)(bf[5])) - std::pow(2, 14)) / std::pow(2, 20);
    this->calib_data.par_p2 = ((double)((int16_t)(bf[8] << 8) | (int16_t)(bf[7])) - std::pow(2, 14)) / std::pow(2, 29);
    this->calib_data.par_p3 = (double)((int8_t)(bf[9])) / std::pow(2, 32);
    this->calib_data.par_p4 = (double)((int8_t)(bf[10])) / std::pow(2, 37);
    this->calib_data.par_p5 = (double)((uint16_t)(bf[12] << 8) | (uint16_t)(bf[11])) / std::pow(2, -3);
    this->calib_data.par_p6 = (double)((uint16_t)(bf[14] << 8) | (uint16_t)(bf[13])) / std::pow(2, 6);
    this->calib_data.par_p7 = (double)((int8_t)(bf[15])) / std::pow(2, 8);
    this->calib_data.par_p8 = (double)((int8_t)(bf[16])) / std::pow(2, 15);
    this->calib_data.par_p9 = (double)((int16_t)(bf[18] << 8) | (int16_t)(bf[17])) / std::pow(2, 48);
    this->calib_data.par_p10 = (double)((int8_t)(bf[19])) / std::pow(2, 48);
    this->calib_data.par_p11 = (double)((int8_t)(bf[20])) / std::pow(2, 65);
    sleep_ms(2);
}

void BMP384::set_power_mode(uint8_t mode)
{
    uint8_t pwr;
    i2c_read(ADDR_PWR_CTRL, 1, &pwr);
    pwr = (pwr & PWR_MODE_MASK) | mode;
    i2c_write(ADDR_PWR_CTRL, 1, &pwr);
    sleep_ms(2);
}

void BMP384::enable_pressure()
{
    uint8_t reg;
    i2c_read(ADDR_PWR_CTRL, 1, &reg);
    reg = (reg & PRESSURE_ENABLE_MASK) | BMP384_PRESSURE_ENABLE;
    i2c_write(ADDR_PWR_CTRL, 1, &reg);
    sleep_ms(2);
}

void BMP384::enable_temperature()
{
    uint8_t reg;
    i2c_read(ADDR_PWR_CTRL, 1, &reg);
    reg = (reg & TEMP_ENABLE_MASK) | BMP384_TEMP_ENABLE;
    i2c_write(ADDR_PWR_CTRL, 1, &reg);
    sleep_ms(2);
}

void BMP384::set_pressure_osr(uint8_t osr)
{
    uint8_t reg;
    i2c_read(ADDR_OSR, 1, &reg);
    reg = (reg & PRESSURE_OSR_MASK) | osr;
    i2c_write(ADDR_OSR, 1, &reg);
    sleep_ms(2);
}

void BMP384::set_temperature_osr(uint8_t osr)
{
    uint8_t reg;
    i2c_read(ADDR_OSR, 1, &reg);
    reg = (reg & TEMP_OSR_MASK) | osr;
    i2c_write(ADDR_OSR, 1, &reg);
}

void BMP384::set_iir_filter(uint8_t coefficient)
{
    uint8_t reg;
    i2c_read(ADDR_CONFIG, 1, &reg);
    reg = (reg & IIR_FILTER_MASK) | coefficient;
    i2c_write(ADDR_CONFIG, 1, &reg);
}

/** \brief Sets up the I2C file handle and connects to a device on the I2C bus.
 * \param[in] address The address of the device on the I2C bus.
 * \param[in] bus_name The name of the bus to open, e.g. /dev/i2c-1.
 */
void BMP384::i2c_init(const int address, const std::string &bus_name)
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
int BMP384::i2c_read(int reg_addr, int bytes, uint8_t *buffer)
{
    return i2c_smbus_read_i2c_block_data(this->handle, reg_addr, bytes, buffer);
}

/** \brief Writes a given number of bytes to a specified register address.
 * \param[in] reg_addr The register address to start writing.
 * \param[in] bytes The number of bytes to write.
 * \param[in] buffer The source of the bytes to be written.
 * \return SMBUS return code.
 */
int BMP384::i2c_write(int reg_addr, int bytes, uint8_t *buffer)
{
    return i2c_smbus_write_i2c_block_data(this->handle, reg_addr, bytes, buffer);
}

int BMP384::get_chip_id()
{
    uint8_t id;
    i2c_read(ADDR_CHIP_ID, 1, &id);
    return id;
}

/** \brief Compute the altitude using the hypsometric formula.
 * The hypsometric formula considing temperature as a variable. This generally make it more
 * accurate but this may not hold true at extremes of temperature.
 * 
 * This function is based on the deriviation by Portland state aerospace society.
 * https://www.researchgate.net/file.PostFileLoader.html?id=5409cac4d5a3f2e81f8b4568&assetKey=AS%3A273593643012096%401442241215893
 * 
 * Height h above sea level is given by
 * 
 * h = ((P0/P)^(LRg) - 1) * (T/L)    equation (8) in the linked derivation.
 * 
 * where P0 = sea level pressure, Pa
 *       P  = measured local pressure, Pa
 *       L  = Lapse rate of temperature with altitude, K/m
 *       R  = gas constant for air, J/kg/K
 *       T  = local measured temperature, Kelvin
 * 
 * \param pressure Measured atmospheric pressure in hPa.
 * \param temperature Measured temperature in degrees C.
 * \param sea_level_pressure Current sea level pressure in hPa.
 */
double BMP384::calculate_altitude(const double pressure, const double temperature, const double sea_level_pressure)
{

    double lapse_rate = 0.00649;              // 0.0065 Kelvin/metre. This is theoretically variable.
    double gas_constant_for_air = 287.053;
    double gravity = 9.8127;                // This theoretically varies with location.

    return ((pow((sea_level_pressure/pressure), (lapse_rate * gas_constant_for_air / gravity))-1)*(temperature + 273.15))/lapse_rate;
}
