#include <stdio.h>
#include <csignal>
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "bmi2.h"
#include "input_parser.h"
#include "bmi270.h"
#include "imu_utils.h"
#include "utils.h"

static int SHUTDOWN = 0;

static std::string config_file = "./arwain.conf";   // Location of default configuration file.
int calibrate_gyro = 0;
int calibrate_mag = 0;
int calibrate_accel = 0;
int n_sec = 15;                                     // Seconds to spend on each calibration activity.

/** \brief Capture the SIGINT signal for clean exit.
 * Sets the global SHUTDOWN flag informing all threads to clean up and exit.
 * \param signal The signal to capture.
 */
static void sigint_handler(int signal)
{
    if (signal == SIGINT)
    {
        std::cout << "\nReceived SIGINT - closing\n" << "\n";
        SHUTDOWN = 1;
    }
}

void calibrate_gyroscope()
{
    std::cout << "Gyroscope calibration: Leave the device completely still for 15 seconds\n";
    vector3 gyro_d;
    vector3 accel_d;
    int gyro_reading_count = 0;
    double gx = 0, gy = 0, gz = 0;
    for (int i = 0; i < n_sec*100; i++)
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
    arwain::config_replace(config_file, "gyro_bias_x", gx_bias);
    arwain::config_replace(config_file, "gyro_bias_y", gy_bias);
    arwain::config_replace(config_file, "gyro_bias_z", gz_bias);
}

void calibrate_magnetometer()
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
    arwain::config_replace(config_file, "mag_bias_x", offset_x);
    arwain::config_replace(config_file, "mag_bias_y", offset_y);
    arwain::config_replace(config_file, "mag_bias_z", offset_z);
    arwain::config_replace(config_file, "mag_scale_x", scale_x);
    arwain::config_replace(config_file, "mag_scale_y", scale_y);
    arwain::config_replace(config_file, "mag_scale_z", scale_z);
}

void calibrate_accelerometer()
{
    // TODO Accelerometer calibration.
    std::cout << "Accelerometer calibration is not yet implemented" << std::endl;
}

void calibrate_gyroscope_temperature()
{
    // Prepare keyboard interrupt signal handler to enable graceful exit.
    std::signal(SIGINT, sigint_handler);

    std::cout << "Starting gyroscope temperature calibration log; press Ctrl+C to quit" << std::endl;

    // Local buffers.
    vector3 gyro_d;
    vector3 _dump;

    // Open file handle to store results.
    std::ofstream output_file{"gyro_temp.txt"};
    output_file << "# time cpu_temp x_bias y_bias z_bias" << std::endl;
    
    while (!SHUTDOWN)
    {
        int gyro_reading_count = 0;
        double gx = 0, gy = 0, gz = 0;

        // Get time.
        auto time = std::chrono::system_clock::now();

        // Take 100 readings over 1 second.
        for (int i=0; i < 30; i++)
        {
            get_bmi270_data(&_dump, &gyro_d);
            gx += gyro_d.x;
            gy += gyro_d.y;
            gz += gyro_d.z;
            gyro_reading_count++;

            delay_ms(10);
        }

        // Measure CPU temperature.
        float cpu_temp = get_bmi270_temperature();

        // Calculate average gyro reading.
        double gx_bias = gx / gyro_reading_count;
        double gy_bias = gy / gyro_reading_count;
        double gz_bias = gz / gyro_reading_count;

        // Write all to file with timestamp.
        output_file << time.time_since_epoch().count() << " " << cpu_temp << " " << gx_bias << " " << gy_bias << " " << gz_bias << std::endl;
        std::cout << time.time_since_epoch().count() << " " << cpu_temp << " " << gx_bias << " " << gy_bias << " " << gz_bias << std::endl;
    }
}

/** \brief Run calibration procedure for the BMI270, putting output in config file.
 * \param argc From 0 to 3 arguments specifying which sensor to calibrate.
 * \param argv Any combination of -mag, -accel, -gyro, or nothing.
 */
int main(int argc, char **argv)
{
    // Detect which sensors to calibrate.
    arwain::InputParser input{argc, argv};
    if (input.contains("-h"))
    {
        std::cout << "Run without arguments to calibrate all sensors\n";
        std::cout << "\n";
        std::cout << "Arguments:\n";
        std::cout << "  -gyro        Calibrate gyroscope\n";
        std::cout << "  -accel       Calibrate accelerometer\n";
        std::cout << "  -mag         Calibrate magnetometer\n";
        std::cout << "  -temp        Log gyroscope offsets against CPU temperature; overrides other arguments\n";
        std::cout << "  -conf        Specify alternate configuration file\n";
        std::cout << "  -h           Show this help text\n";
        std::cout << "\n";
        std::cout << "Example usage:\n";
        std::cout << "  ./calib -mag -gyro\n";
        std::cout << "  to calibrate magnetometer and gyroscope only\n";
        std::cout << "\n";
        std::cout << "Error codes:\n";
        std::cout << "   0           Successful execution\n";
        std::cout << "  -1           Sensor couldn't start" << std::endl;
        return 1;
    }

    // Start sensors.
    if (init_bmi270(1, "no_calib_here") != 0)
    {
        std::cout << "Sensor failed to start" << std::endl;
        return -1;
    }

    if (input.contains("-conf"))
    {
        config_file = input.getCmdOption("-conf");
    }
    if (input.contains("-temp"))
    {
        calibrate_gyroscope_temperature();
        return 0;
    }
    if (input.contains("-gyro"))
    {
        calibrate_gyroscope();
    }
    if (input.contains("-mag"))
    {
        calibrate_magnetometer();
    }
    if (input.contains("-accel"))
    {
        calibrate_accelerometer();
    }

    return 0;
}

