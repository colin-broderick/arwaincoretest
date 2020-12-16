#include "quaternions.h"
#include "stance.h"
#include "pi_utils.h"

#include <csignal>
#include <iostream>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>
#include <deque>
#include <array>
#include <math.h>
#include <vector>
#include <thread>
#include <mutex>


// IMU data buffers.
std::deque<std::array<float, 6>> imu_full_buffer;
std::deque<std::array<float, 6>> imu_stepped_buffer;
std::deque<std::array<float, 3>> vel_full_buffer;


void efaroe_step(std::array<float, 3> gyro);

// Kill flag for all threads.
int shutdown = 0;

unsigned int step_size = 50;
unsigned int window_size = 200;


std::mutex full_buffer_lock;
std::mutex stepped_buffer_lock;
std::mutex vel_buffer_lock;


void bmi270_reader()
{
    // Initialize the IMU.
    std::string path = "../calib.txt";
    if (init_bmi270(1, path) != 0)
    {
        printf("Node failed to start\n");
        exit(1);
    }

    vec_scaled_output accel_data;
    vec_scaled_output gyro_data;
    vec_scaled_output mag_data;

    // Start reading the IMU into the accel_full_buffer.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(50);

    int count = 0;

    while (!shutdown)
    {
        // Read current sensor values.
        get_bmi270_data(&accel_data, &gyro_data);

        // Add new reading to end of buffer, and remove oldest reading from start of buffer.
        full_buffer_lock.lock();
        imu_full_buffer.push_back(std::array<float, 6>{
            accel_data.x,
            accel_data.y,
            accel_data.z,
            gyro_data.x,
            gyro_data.y,
            gyro_data.z
        });
        imu_full_buffer.pop_front();
        full_buffer_lock.unlock();

        // Perform an efaroe step to update orientation.
        efaroe_step(std::array<float, 3>{
            gyro_data.x,
            gyro_data.y,
            gyro_data.z
        });
   
        count++;
        if (count % window_size == 0)
        {
            std::cout << count << std::endl;
        }
        
        // Wait until the next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
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
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(1000/window_size);
    
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
        std::this_thread::sleep_until(time);
    }
}


int transmit_lora()
{
    // TODO: Set up radio.

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(1000);

    while (!shutdown)
    {
        // std::cout << "LoRa tick\n" << std::endl;
        // TODO: Read all relevant data, respecting mutex locks.
        // TODO: Build data packet for transmission.
        // TODO: Send transmission.
        
        // TODO: Maybe a read loop?
        
        // Wait until next tick
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    return 1;
}

int predict_velocity()
{
    // TODO: Set up NPU and feed in model.

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(50);

    std::array<float, 3> vel;

    while (!shutdown)
    {
        // TODO: Make velocity prediction
        vel = std::array<float, 3>{0, 0, 0};

        // Add velocity to buffer.
        vel_buffer_lock.lock();
        vel_full_buffer.push_back(vel);
        vel_full_buffer.pop_front();
        vel_buffer_lock.unlock();

        // TODO Integrate new vel onto position. Keep a position history or just the current value?

        // Wait until next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    return 1;
}

void sigint_handler(int signal)
{
    std::cout << "Called quit\n" << std::endl;
    shutdown = 1;
}

int main()
{
    // Prepare keyboard interrupt signal handler to enable graceful exit.
    std::signal(SIGINT, sigint_handler);

    // Preload buffers
    for (unsigned int i = 0; i < window_size; i++)
    {
        imu_full_buffer.push_back(std::array<float, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        imu_stepped_buffer.push_back(std::array<float, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        vel_full_buffer.push_back(std::array<float, 3>{0.0, 0.0, 0.0});
    }

    // Start threads
    std::thread imu_full_thread(bmi270_reader);
    std::thread imu_step_thread(imu_stepper);
    std::thread velocity_prediction(predict_velocity);
    std::thread fall_detection(run_fall_detect);
    std::thread stance_detection(run_stance_detect);
    std::thread radio_thread(transmit_lora);

    // Spin until shutdown signal received.
    std::chrono::milliseconds delay(1000);
    while (!shutdown)
    {
        std::this_thread::sleep_for(delay);
    }

    // Wait for all threads to terminate.
    imu_full_thread.join();
    imu_step_thread.join();
    velocity_prediction.join();
    fall_detection.join();
    stance_detection.join();
    radio_thread.join();
}
