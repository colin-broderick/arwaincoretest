#include <vector>
#include <sstream>
#include <fstream>
#include <iostream>
#include <mutex>
extern "C"
{
    #include <i2c/smbus.h>
    #include <linux/i2c-dev.h>
}
#include <fcntl.h>
#include <sys/ioctl.h>

#include "bmi270.hpp"

BMI270::BMI270(const int i2c_address, const std::string& i2c_bus)
{
    init_bmi270(0, "none", i2c_bus);
}

void BMI270::read_IMU()
{
    get_bmi270_data(&(this->acce), &(this->gyro));
    accelerometer_x = this->acce.x;
    accelerometer_y = this->acce.y;
    accelerometer_z = this->acce.z;
    gyroscope_x = this->gyro.x;
    gyroscope_y = this->gyro.y;
    gyroscope_z = this->gyro.z;
}

double BMI270::read_temperature()
{
    // TODO
    return 0;
}


int init_bmi270(int mag_enabled, const std::string& calib_file, const std::string& i2c_bus)
{
    float g = 9.8067;
    float pi = 3.14159265359;
    float max_value = 32767.0;
    float acc_range = g*16.0;
    float gyr_range = 2000.0*(pi/180.0);
    
    acc_scale = acc_range/max_value;
    gyr_scale = gyr_range/max_value;

    acce.type = BMI2_ACCEL;
    gyro.type = BMI2_GYRO;
    magn.type = BMI2_AUX;
	/*To enable I2C interface*/
    bmi270.read = bmi270_reg_read;
    bmi270.write = bmi270_reg_write;
    bmi270.aux_man_en = BMI2_TRUE;
    bmi270.delay_us = delay_us;
    bmi270.read_write_len = 32;
    bmi270.intf = BMI2_I2C_INTERFACE;
    bmi270.dev_id = BMI2_I2C_SEC_ADDR;
    i2c_init(bmi270.dev_id, bmi_file_i2c, i2c_bus);
    	
	// Initialise device
    int bmi_rslt = bmi270_init(&bmi270);
    if (bmi_rslt == 0)
    {
	    std::cout << "IMU initialised" << std::endl;
    }
    else
    {
        std::cout << "IMU init failed. Error " << bmi_rslt << std::endl;
        return 1;
    }

    // Enable accelerometer
    uint8_t sensorList[] = {BMI2_ACCEL, BMI2_GYRO, BMI2_AUX};
    uint8_t sensorOffList[] = {BMI2_GYRO_SELF_OFF};
    bmi2_sensor_enable(&sensorList[0], 1, &bmi270);
    
    // Configure accelerometer
    struct bmi2_sens_config accelerometerConfig;
    accelerometerConfig.type = BMI2_ACCEL;
    accelerometerConfig.cfg.acc.odr = BMI2_ACC_ODR_400HZ;
    accelerometerConfig.cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
    accelerometerConfig.cfg.acc.filter_perf = 0;
    accelerometerConfig.cfg.acc.range = BMI2_ACC_RANGE_16G;
    bmi2_set_sensor_config(&accelerometerConfig, 1, &bmi270);
    bmi2_do_crt(&bmi270);

    // Enable gyroscope
    bmi2_sensor_enable(&sensorList[1], 1, &bmi270);
        
    // Configure gyroscope
    struct bmi2_sens_config gyroscopeConfig;
    gyroscopeConfig.type = BMI2_GYRO;
    gyroscopeConfig.cfg.gyr.odr = BMI2_GYR_ODR_400HZ;
    gyroscopeConfig.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    gyroscopeConfig.cfg.gyr.filter_perf = 0;
    gyroscopeConfig.cfg.gyr.ois_range = BMI2_GYR_OIS_2000;
    gyroscopeConfig.cfg.gyr.range = BMI2_GYR_RANGE_2000;
    gyroscopeConfig.cfg.gyr.noise_perf = 1;
    bmi2_set_sensor_config(&gyroscopeConfig, 1, &bmi270);

    // Disable self offset correction
    bmi2_sensor_disable(&sensorOffList[0], 1, &bmi270);
    
    // Disable all offset compensation
    // uint8_t reg_data = 0;
    // int8_t rslt = bmi2_get_regs(BMI2_GYR_OFF_COMP_6_ADDR, &reg_data, 1, &bmi270);
    // if (rslt == BMI2_OK)
    // {
    //     reg_data = BMI2_SET_BITS(reg_data, BMI2_GYR_OFF_COMP_EN, 0);
    //     rslt = bmi2_set_regs(BMI2_GYR_OFF_COMP_6_ADDR, &reg_data, 1, &bmi270);
    // }
    // if (rslt != BMI2_OK)
    // {
    //     std::cout << "Failed to set reg" << std::endl;
    // }

    //Magnetometer setup
    if (mag_enabled)
    {	
        // Enable magnetometer
        bmi2_sensor_enable(&sensorList[2], 1, &bmi270);
        
        // Configure gyroscope
        struct bmi2_sens_config magnetometerConfig;
        magnetometerConfig.type = BMI2_AUX;
        magnetometerConfig.cfg.aux.aux_en = BMI2_ENABLE;
        magnetometerConfig.cfg.aux.manual_en = BMI2_TRUE;
        magnetometerConfig.cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS; // I2C address of BMM150
        magnetometerConfig.cfg.aux.read_addr = 0x40;	// Address of the first read address
        magnetometerConfig.cfg.aux.man_rd_burst = BMI2_AUX_READ_LEN_3;	// Total number of registers to be read for manual burst (0x40 to 0x71)
        magnetometerConfig.cfg.aux.offset = 2;			// Offset for auto burst read
        magnetometerConfig.cfg.aux.offset = 2;			// Offset for auto burst read
        magnetometerConfig.cfg.aux.aux_rd_burst = BMI2_AUX_READ_LEN_3;	// Size of auto burst read
        magnetometerConfig.cfg.aux.fcu_write_en = BMI2_ENABLE;
        magnetometerConfig.cfg.aux.odr = 1;
        bmi2_set_sensor_config(&magnetometerConfig, 1, &bmi270);
        
        bmm150.dev_id = BMM150_DEFAULT_I2C_ADDRESS;
        bmm150.intf = BMM150_I2C_INTF;
        bmm150.read = bmm150_reg_read;
        bmm150.write = bmm150_reg_write;
        bmm150.delay_ms = delay_ms;
        
        int bmm_rslt = bmm150_init(&bmm150);
        if (bmm_rslt == 0)
        {
            std::cout << "Magnetometer initialised" << std::endl;
        }
        else
        {
            std::cout << "Magnetometer init failed. Error " << bmm_rslt << std::endl;
            return 1;
        }
        
        bmm150.settings.pwr_mode = BMM150_NORMAL_MODE;
        bmm150_set_op_mode(&bmm150);
        bmm150.settings.preset_mode = BMM150_PRESETMODE_REGULAR;
        bmm150_set_presetmode(&bmm150);
        bmm150.settings.data_rate = BMM150_DATA_RATE_30HZ;
        bmm150_set_sensor_settings(BMM150_DATA_RATE_SEL, &bmm150);
        
        read_calib_data(calib_file);
        
    }
    
    return 0;
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
    int8_t ret = i2c_smbus_read_i2c_block_data(bmi_file_i2c, reg_addr, length, reg_data);
    return ret < 0;
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
    int8_t ret = i2c_smbus_write_i2c_block_data(bmi_file_i2c, reg_addr, length, reg_data);
    return ret < 0;
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

int i2c_init(const int address, int& file_i2c, const std::string& i2c_bus)
{
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)(i2c_bus.c_str());
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

void read_calib_data(const std::string& path)
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

int get_bmi270_data(struct vector3 *acc, struct vector3 *gyr)
{
    int rslt;
    rslt = bmi2_get_sensor_data(&acce, 1, &bmi270);
    if (rslt != BMI2_OK)
    {
    	std::cout << "Acc Error num " << rslt << std::endl;
    	return 1;
    }
    rslt = bmi2_get_sensor_data(&gyro, 1, &bmi270);
    if (rslt != BMI2_OK)
    {
    	std::cout << "Gyr Error num " << rslt << std::endl;
    	return 1;
    }

    acc->x = acce.sens_data.acc.x * acc_scale;
    acc->y = acce.sens_data.acc.y * acc_scale;
    acc->z = acce.sens_data.acc.z * acc_scale;
    
    gyr->x = gyro.sens_data.gyr.x * gyr_scale;
    gyr->y = gyro.sens_data.gyr.y * gyr_scale;
    gyr->z = gyro.sens_data.gyr.z * gyr_scale;
    
    return 0;
}
