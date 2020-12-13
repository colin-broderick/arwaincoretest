#include "BufferedSerial.h"
#include "I2C.h"
#include "Kernel.h"
#include "PinNames.h"
#include "ThisThread.h"
#include "mbed.h"
#include "LSM6DS33.h"
#include "mbed_thread.h"
#include "quaternions.h"
#include "stance.h"

#include <iostream>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>
#include <deque>
#include <array>
#include <math.h>
#include <vector>


// IMU data buffers.
std::deque<std::array<float, 6>> imu_full_buffer;
std::deque<std::array<float, 6>> imu_stepped_buffer;
std::deque<std::array<float, 3>> vel_full_buffer;


void efaroe_step(std::array<float, 3> gyro);


// Open serial connection for debugging.
BufferedSerial serial(USBTX, USBRX);

// Kill flag for all threads.
int shutdown = 0;

unsigned int step_size = 50;
unsigned int window_size = 200;

Mutex full_buffer_lock;
Mutex stepped_buffer_lock;
Mutex vel_buffer_lock;


void imu_reader()
{
    serial.set_baud(9600);
    int address = 0x6a << 1;


    // Initialize the IMU.
    LSM6DS33 sensor(p28, p27, address);
    unsigned int response = sensor.begin(
        LSM6DS33::G_SCALE_245DPS,
        LSM6DS33::A_SCALE_8G,
        LSM6DS33::G_ODR_104,
        LSM6DS33::A_ODR_104
    );

    // Check the IMU responded.
    if (response != 0x69)
    {
        serial.write("No reponse from IMU\n", 20);
        exit(1);
    }
    else
    {
        serial.write("Connected IMU\n", 14);
    }

    // Start reading the IMU into the accel_full_buffer.
    auto time = Kernel::Clock::now();
    auto interval = (1000 / window_size) * 1ms;

    int count = 0;

    while (!shutdown)
    {
        // Read current sensor values.
        sensor.readAccel();
        sensor.readGyro();

        // Add new reading to end of buffer, and remove oldest reading from start of buffer.
        full_buffer_lock.lock();
        imu_full_buffer.push_back(std::array<float, 6>{
            sensor.ax,
            sensor.ay,
            sensor.az,
            sensor.gx,
            sensor.gy,
            sensor.gz
        });
        imu_full_buffer.pop_front();
        full_buffer_lock.unlock();

        // Perform an efaroe step to update orientation.
        efaroe_step(std::array<float, 3>{
            sensor.gx,
            sensor.gy,
            sensor.gz
        });
   
        count++;
        if (count % window_size == 0)
        {
            printf("%i\n", count);
        }
        
        // Wait until the next tick.
        time = time + interval;
        ThisThread::sleep_until(time);
    }
}


class efaroe
{
    public:

        std::array<float, 3> gyro_bias;
        std::array<float, 3> emf;
        float zeta;
        int last_read;
        int use_mag;
        float gyro_error;
        float true_error;
        float beta;
        quaternion q;
        int conv_count;
        float uk_dip = -67*3.14159/180;

        efaroe(quaternion initial_quaternion, std::array<float, 3> gyro_bias, float gyro_error, int use_mag)
        {
            this->gyro_bias = gyro_bias;
            this->emf = std::array<float, 3>{cos(uk_dip), 0, sin(uk_dip)};
            this->zeta = sqrt(3) * 1e-2;
            this->last_read = 0;
            this->use_mag = use_mag;

            this->q = initial_quaternion;
            this->gyro_error = gyro_error;
            this->true_error = gyro_error;
            this->beta = sqrt(3) * this->gyro_error;
            this->conv_count = -1;
        }

        void update(int reading_time, std::array<float, 6> imu)
        {
            if (conv_count >= 0)
            {
                conv_count = conv_count -1;
                if (conv_count == 0)
                {
                    gyro_error = true_error;
                    beta = sqrt(3) * gyro_error;
                }
            }

            if (last_read == 0)
            {
                last_read = reading_time;
                return;
            }
            else {
                float dt = reading_time - last_read;
                if (dt > 1)
                {
                    dt = 1;
                }
                last_read = reading_time;
            }

            double acce_mag = sqrt(imu[0]*imu[0]+imu[1]*imu[1]+imu[2]*imu[2]);
            std::array<double, 3> acce{
                imu[0] / acce_mag,
                imu[1] / acce_mag,
                imu[2] / acce_mag
            };
            std::array<double, 3> jac_a{
                q.getIm_i() * q.getIm_k() * 2 - q.getRe() * q.getIm_j() * 2, 
                q.getRe() * q.getIm_i() * 2 + q.getIm_j() * q.getIm_k() * 2,
                1 - (2 * q.getIm_i() * q.getIm_i()) - (2 * q.getIm_j() * q.getIm_j())
            };


        }

};


// void cross(std::array<double, 3> v1, std::array<double, 3> v2)
// {
//     std::vector<std::vector<std::array<float, 3>>> product{
//         std::array<float, 3>{0, 0, 0},
//         std::array<float, 3>{0, 0, 0},
//         std::array<float, 3>{0, 0, 0}
//     };
// }


void efaroe_step(std::array<float, 3> gyro)
{

}


void imu_stepper()
{
    auto time = Kernel::Clock::now();
    auto interval = 1000/(window_size/step_size) * 1ms;


    while (!shutdown)
    {
        // Copy existing full buffer into stepped buffer
        full_buffer_lock.lock();
        stepped_buffer_lock.lock();
        imu_stepped_buffer = imu_full_buffer;
        full_buffer_lock.unlock();
        stepped_buffer_lock.unlock();

        // Wait until the next tick.
        time = time + interval;
        ThisThread::sleep_until(time);
    }
}

int transmit_lora()
{
    // TODO: Set up radio.

    // Set up timing.
    auto time = Kernel::Clock::now();
    auto interval = 1000 * 1ms;

    while (!shutdown)
    {
        // TODO: Read all relevant data, respecting mutex locks.
        // TODO: Build data packet for transmission.
        // TODO: Send transmission.
        
        // TODO: Maybe a read loop?
        
        // Wait until next tick
        time = time + interval;
        ThisThread::sleep_until(time);
    }

    return 1;
}

int predict_velocity()
{
    // TODO: Set up NPU and feed in model.

    // Set up timing.
    auto time = Kernel::Clock::now();
    auto interval = 50 * 1ms;

    while (!shutdown)
    {
        // TODO: Make velocity prediction
        std::array<float, 3> vel = std::array<float, 3>{0, 0, 0};

        // Add velocity to buffer.
        vel_buffer_lock.lock();
        vel_full_buffer.push_back(vel);
        vel_full_buffer.pop_front();
        vel_buffer_lock.unlock();

        // TODO Integrate new vel onto position. Keep a position history or just the current value?

        time = time + interval;
        ThisThread::sleep_until(time);
    }

    return 1;
}

int main()
{
    // Preload buffers
    for (unsigned int i = 0; i < window_size; i++)
    {
        imu_full_buffer.push_back(std::array<float, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        imu_stepped_buffer.push_back(std::array<float, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        vel_full_buffer.push_back(std::array<float, 3>{0.0, 0.0, 0.0});
    }

    // Start the IMU thread
    Thread imu_full_thread;
    imu_full_thread.start(imu_reader);

    // Start thread for binning imu in packets every step_size steps.
    Thread imu_step_thread;
    imu_step_thread.start(imu_stepper);

    Thread velocity_prediction;
    velocity_prediction.start(predict_velocity);

    Thread fall_detection;
    fall_detection.start(run_fall_detect);

    Thread stance_detection;
    stance_detection.start(run_stance_detect);

    Thread radio_thread;
    radio_thread.start(transmit_lora);

    // Spin until shutdown signal received.
    while (!shutdown)
    {
        ThisThread::sleep_for(1s);
    }

    // Wait for all threads to terminate.
    imu_full_thread.join();
    imu_step_thread.join();
    velocity_prediction.join();
    fall_detection.join();
    stance_detection.join();
    radio_thread.join();
}
