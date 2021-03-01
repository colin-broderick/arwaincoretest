#include <stdio.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "bmi2.h"
#include "input_parser.h"
#include "bmi270.h"
#include "imu_utils.h"
#include "utils.h"

static std::string config_file = "./arwain.conf";

/** \brief Run calibration procedure for the BMI270, putting output in config file.
 * \param argc From 0 to 3 arguments specifying which sensor to calibrate.
 * \param argv Any combination of -mag, -accel, -gyro, or nothing.
 */
int main(int argc, char **argv)
{
    // Detect which sensors to calibrate.
    int calibrate_gyro = 0, calibrate_mag = 0, calibrate_accel = 0;
    arwain::InputParser input{argc, argv};
    if (input.contains("-h"))
    {
        std::cout << "Run without arguments to calibrate all sensors\n";
        std::cout << "\n";
        std::cout << "Arguments:\n";
        std::cout << "  -gyro        Calibrate gyroscope\n";
        std::cout << "  -accel       Calibrate accelerometer\n";
        std::cout << "  -mag         Calibrate magnetometer\n";
        std::cout << "  -conf        Specify alternate configuration file\n";
        std::cout << "  -h           Show this help text\n";
        std::cout << "\n";
        std::cout << "Example usage:\n";
        std::cout << "  ./calib -mag -gyro\n";
        std::cout << "  to calibrate magnetometer and gyroscope only\n";
        std::cout << "\n";
        std::cout << "Error codes:\n";
        std::cout << "   1           Successful execution\n";
        std::cout << "  -1           Sensor couldn't start\n";
        return 1;
    }
    if (input.contains("-conf"))
    {
        config_file = input.getCmdOption("-conf");
    }
    if (input.contains("-gyro"))
    {
        calibrate_gyro = 1;
    }
    if (input.contains("-mag"))
    {
        calibrate_mag = 1;
    }
    if (input.contains("-accel"))
    {
        calibrate_accel = 1;
    }
    if (calibrate_accel == 0 && calibrate_gyro == 0 && calibrate_mag == 0)
    {
        calibrate_gyro = 1;
        calibrate_mag = 1;
        calibrate_accel = 1;
    }

    // Start sensors.
    if (init_bmi270(1, "no_calib_here") != 0)
    {
        std::cout << "Sensor failed to start\n";
        return -1;
    }

    // Time to spend on each test.    
    int n_sec = 15;

    // Calibrate gyroscope.
    if (calibrate_gyro)
    {
        std::cout << "Gyroscope calibration: Leave the device completely still for 15 seconds\n";
        vector3 gyro_d;
        vector3 accel_d;
        int gyro_reading_count = 0;
        double gx = 0, gy = 0, gz = 0;
        for(int i = 0; i < n_sec*100; i++)
        {
            get_bmi270_data(&accel_d, &gyro_d);
            gx += gyro_d.x;
            gy += gyro_d.y;
            gz += gyro_d.z;
            gyro_reading_count += 1;
            if (i%100 == 0)
            {
                std::cout << "T = " << (int)(i/100) << "\n";
            }
            delay_ms(10);
        }
        double gx_bias = gx/gyro_reading_count;
        double gy_bias = gy/gyro_reading_count;
        double gz_bias = gz/gyro_reading_count;
        config_replace(config_file, "gyro_bias_x", gx_bias);
        config_replace(config_file, "gyro_bias_y", gy_bias);
        config_replace(config_file, "gyro_bias_z", gz_bias);
    }
    
    // Calibrate magneometer.
    if (calibrate_mag)
    {
        vector3 mag_d;
        std::vector<float> x, y, z;
        std::cout << "Magnetometer calibration: Roll the device through all orientations for a period of 15 seconds\n";
        for(int i=0; i<n_sec*100; i++)
        {
            get_bmm150_data(&mag_d);
            x.push_back(mag_d.x);
            y.push_back(mag_d.y);
            z.push_back(mag_d.z);
            if (i%100 == 0)
            {
                std::cout << "T = " << (int)(i/100) << "\n";
            }
            delay_ms(10);
        }
        
        float max_x = -10;
        float min_x = 10;
        float max_y = -10;
        float min_y = 10;
        float max_z = -10;
        float min_z = 10;
        
        for (auto i = x.begin(); i != x.end(); ++i)
        {
            if (*i < min_x)
                min_x = *i;
            if (*i > max_x)
                max_x = *i;
        }
        
        for (auto i = y.begin(); i != y.end(); ++i)
        {
            if (*i < min_y)
                min_y = *i;
            if (*i > max_y)
                max_y = *i;
        }
        
        for (auto i = z.begin(); i != z.end(); ++i)
        {
            if (*i < min_z)
                min_z = *i;
            if (*i > max_z)
                max_z = *i;
        }

        float offset_x = (max_x + min_x) / 2;
        float offset_y = (max_y + min_y) / 2;
        float offset_z = (max_z + min_z) / 2;
        
        float avg_delta_x = (max_x - min_x) / 2;
        float avg_delta_y = (max_y - min_y) / 2;
        float avg_delta_z = (max_z - min_z) / 2;

        float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;

        float scale_x = avg_delta / avg_delta_x;
        float scale_y = avg_delta / avg_delta_y;
        float scale_z = avg_delta / avg_delta_z;

        // Write all the new values to config file.
        config_replace(config_file, "mag_bias_x", offset_x);
        config_replace(config_file, "mag_bias_y", offset_y);
        config_replace(config_file, "mag_bias_z", offset_z);
        config_replace(config_file, "mag_scale_x", scale_x);
        config_replace(config_file, "mag_scale_y", scale_y);
        config_replace(config_file, "mag_scale_z", scale_z);
    }
    
    if (calibrate_accel)
    {
        // TODO Accelerometer calibration.
    }

    return 0;
}
