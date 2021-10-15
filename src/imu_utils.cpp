#include <stdio.h>
#include <stdint.h>
#include <unistd.h>		
#include <time.h>		
#include <fcntl.h>	
#include <vector>			
#include <sys/ioctl.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <mutex>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <smbus.h>
}		
#include "imu_utils.hpp"

std::mutex I2C_LOCK;
int bmi_file_i2c = -1;
struct timespec tim, tim_r;

struct bmi2_dev bmi270;
struct bmm150_dev bmm150;

vector3 mag_calib_offset, mag_calib_scale;

int i2c_init(const int address, int& file_i2c)
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

float get_bmi270_temperature()
{
    uint8_t buffer[2] = {0, 0};
    bmi270_reg_read(BMI2_I2C_SEC_ADDR, 0x22, buffer, 2);
    return (((int)buffer[1] << 8) | ((int)buffer[0]))*0.001953f + 23.0f;
}






void read_calib_data(std::string path)
{
    //Read calibration
    std::ifstream source;
    source.open(path, std::ios_base::in);
    mag_calib_offset.x = 0;
    mag_calib_offset.y = 0;
    mag_calib_offset.z = 0;
    mag_calib_scale.x = 1;
    mag_calib_scale.y = 1;
    mag_calib_scale.z = 1;

    if (!source)
    {
	    std::cout << "No calibration file found, magnetometer will only use on-board calibration" << std::endl;
    }
    else
    {
        std::vector<float> calib_data;
        std::string line;
        std::getline(source, line); // Skip header
        std::getline(source, line);
        std::stringstream linestream(line);
        std::string value;

        while(getline(linestream,value,','))
        {
            calib_data.push_back(std::stof(value));
        }

        mag_calib_offset.x = calib_data[0];
        mag_calib_offset.y = calib_data[1];
        mag_calib_offset.z = calib_data[2];
        mag_calib_scale.x = calib_data[3];
        mag_calib_scale.y = calib_data[4];
        mag_calib_scale.z = calib_data[5];
    }
    source.close();
}

void delay_us(uint32_t period)
{
    
    // tim.tv_nsec = period*1000;
    usleep(period);
    // nanosleep(&tim, &tim_r);
    
    /* Wait for a period amount of us*/
}

void delay_ms(uint32_t period)
{
    
    delay_us(period*1000);
    
    /* Wait for a period amount of ms*/
}

/*!
 *  @brief Function for writing the IMU's registers through I2C bus.
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
int8_t bmi270_reg_write(uint8_t i2c_addr, uint8_t reg_addr, const uint8_t *reg_data, uint16_t length)
{
    std::lock_guard<std::mutex> lock{I2C_LOCK};
    int8_t ret = i2c_smbus_write_i2c_block_data(bmi_file_i2c, reg_addr, length, reg_data);
    return ret < 0;
}

/*!
 *  @brief Function for reading the IMU's registers through I2C bus.
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
int8_t bmi270_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    std::lock_guard<std::mutex> lock{I2C_LOCK};
    int8_t ret = i2c_smbus_read_i2c_block_data(bmi_file_i2c, reg_addr, length, reg_data);
    return ret < 0;
}



/*!
 *  @brief Function for writing the Magnetometers's registers through I2C bus.
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
int8_t bmm150_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    return bmi2_write_aux_man_mode(reg_addr, reg_data, length, &bmi270);
}

/*!
 *  @brief Function for reading the Magnetometers's registers through I2C bus.
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
int8_t bmm150_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    return bmi2_read_aux_man_mode(reg_addr, reg_data, length, &bmi270);
}


