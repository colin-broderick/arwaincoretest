/*!
 *  @brief Example shows basic setup of sensor which includes following
 *      Initialization of the interface.
 *      performing the sensor initialization.
 */
 
#include <stdio.h>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}
#include <fcntl.h>
#include <sys/ioctl.h>
#include <mutex>
#include <deque>
#include <chrono>
#include <thread>
#include <time.h>
#include <math.h>
#include <unistd.h>
#include <iostream>

#include "bmp280.hpp"
// #include "imu_utils.hpp"

int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
void print_rslt(const char api_name[], int8_t rslt);

extern int SHUTDOWN;
extern std::deque<std::array<double, 3>> PRESSURE_BUFFER;
int bmp_file_i2c;
struct timespec tim2, tim_r2;

double sea_level_pressure;

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
 */
double altitude_from_pressure_and_temperature(const double pressure, const double temperature)
{

    double lapse_rate = 0.00649;              // 0.0065 Kelvin/metre. This is theoretically variable.
    double gas_constant_for_air = 287.053;
    double gravity = 9.8127;                // This theoretically varies with location.

    return ((pow((sea_level_pressure/pressure), (lapse_rate * gas_constant_for_air / gravity))-1)*(temperature + 273.15))/lapse_rate;
}

static void delay_us(uint32_t period)
{
    
    // tim.tv_nsec = period*1000;
    usleep(period);
    // nanosleep(&tim, &tim_r);
    
    /* Wait for a period amount of us*/
}

static void delay_ms(uint32_t period)
{
    
    delay_us(period*1000);
    
    /* Wait for a period amount of ms*/
}

static int i2c_init(const int address, int& file_i2c)
{
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		std::cout << "Failed to open I2C bus" << std::endl;
	}

    if (ioctl(file_i2c, I2C_SLAVE, address) < 0)
    {
        std::cout << "Failed to connect to I2C address " << std::hex << address << std::endl;
    }

	return file_i2c;
}

int init_bmp280(bmp280_dev& bmp, bmp280_config &conf, bmp280_uncomp_data& uncomp_data, const double sealevelpressure)
{
    int8_t rslt;

    sea_level_pressure = sealevelpressure;

    /* Map the delay function pointer with the function responsible for implementing the delay */
    bmp.delay_ms = delay_ms;

    /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
    bmp.dev_id = BMP280_I2C_ADDR_PRIM;

    /* Select the interface mode as I2C */
    bmp.intf = BMP280_I2C_INTF;

    /* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
    bmp.read = i2c_reg_read;
    bmp.write = i2c_reg_write;

    // Create i2c file handle.
    i2c_init(bmp.dev_id, bmp_file_i2c);

    // Attempt to initialize device.
    rslt = bmp280_init(&bmp);
    print_rslt(" bmp280_init status", rslt);

    // Read current configuration.
    rslt = bmp280_get_config(&conf, &bmp);
    print_rslt(" bmp280_get_config status", rslt);

    // Configure temp oversampling, filter coeff, and ODR
    conf.filter = BMP280_FILTER_COEFF_16;
    
    // Configure pressure oversampling
    conf.os_pres = BMP280_OS_16X;
    conf.os_temp = BMP280_OS_16X;

    // Configure ODR at 1 Hz
    conf.odr = BMP280_ODR_125_MS;
    
    // Apply configuration
    rslt = bmp280_set_config(&conf, &bmp);
    print_rslt(" bmp280_set_config status", rslt);

    // Set normal power mode
    rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
    print_rslt(" bmp280_set_power_mode status", rslt);

    return 0;
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    /* Implement the I2C write routine according to the target machine. */
    // return -1;

    /* Implemented for raspberry pi using smbus */
    int8_t ret = i2c_smbus_write_i2c_block_data(bmp_file_i2c, reg_addr, length, reg_data);
    return ret < 0;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{

    /* Implement the I2C read routine according to the target machine. */
    // return -1;

    // Implemented for raspberry pi using smbus */
    int8_t ret = i2c_smbus_read_i2c_block_data(bmp_file_i2c, reg_addr, length, reg_data);
    return ret < 0;
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(const char api_name[], int8_t rslt)
{
    if (rslt != BMP280_OK)
    {
        printf("%s\t", api_name);
        if (rslt == BMP280_E_NULL_PTR)
        {
            printf("Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
            printf("Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
            printf("Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
            printf("Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
            printf("Error [%d] : Unknown error code\r\n", rslt);
        }
    }
}
