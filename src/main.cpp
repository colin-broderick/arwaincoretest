/*
Code style conventions
===============================================================================

I have attempted to use the following conventions throughout this project.
If anything doesn't follow convention, feel free to change it

ENUM           Enumeration types are named in all caps.
EnumElement    Enumerations are named in Pascal case.
Class          Classes are named in Pascal Case and represent complex types.
structType     Structs are reserved for simple data types, named in camel case.
getMethod      Class and struct methods are named in camel case.
GLOBAL_VAR     File- or project-global variables are named in upper snake case.
local_var      Local variables are named in lower snake case.
a_function     Functions are named in lower snake case.

Data integrity
===============================================================================

These rules are established to ensure that time-sensitive operations can execute
successfully and on time, and that data is never inadvertently changed. Deviation
from these rules should be accompanied by a comment clearly indiciating why.

- Anything accessed by more than one thread should be protected by a mutex.
- Mutex locks should be established only for the duration of a read or write and
  then released immediately. No processing should be done while data is locked.
- Any non-primitive data types should be passed by const reference unless a copy
  is explicitly required.

*/

#include <algorithm>
#include <csignal>
#include <iostream>
#include <sstream>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <string>
#include <deque>
#include <stdint.h>
#include <array>
#include <math.h>
#include <vector>
#include <thread>
#include <mutex>
#include <fstream>
#include <experimental/filesystem>
#include <zmq.h>
#include <string.h>

#include "arwain_torch.h"
#include "quaternions.h"
#include "stance.h"
#include "imu_utils.h"
#include "madgwick.h"
#include "efaroe.h"
#include "math_util.h"
#include "input_parser.h"
#include "bin_log.h"
#include "indoor_positioning_wrapper.h"
#include "filter.h"

#include "lora.h"
#include "packet.h"
#include "half.h"
// #include "rknn_api.h"

// Time intervals, all in milliseconds.
static unsigned int IMU_READING_INTERVAL = 5;
static unsigned int VELOCITY_PREDICTION_INTERVAL = 500;
static unsigned int LORA_TRANSMISSION_INTERVAL = 10;
static unsigned int STANCE_DETECTION_INTERVAL = 1000;
static unsigned int INDOOR_POSITIONING_INTERVAL = 50;

// Buffer sizes.
static unsigned int POSITION_BUFFER_LEN = 200;
static unsigned int MAG_BUFFER_LEN = 200;
static unsigned int VELOCITY_BUFFER_LEN = 200;
static unsigned int ORIENTATION_BUFFER_LEN = 200;
static unsigned int IMU_BUFFER_LEN = 200;
static unsigned int IPS_BUFFER_LEN = 50;
static unsigned int LORA_MESSAGE_LENGTH = 8;

// Globally accessible configuration and status.
static arwain::Configuration CONFIG;
static arwain::Status STATUS;

// Default config file locations.
std::string CONFIG_FILE = "./arwain.conf";

// Flags for whether to produce various log outputs.
int LOG_TO_STDOUT = 0;
int LOG_TO_FILE = 0;
int NO_INFERENCE = 0;
int NO_IMU = 0;

// Name for data folder
static std::string FOLDER_DATE_STRING;

// Data buffers.
std::deque<std::array<double, 6>> IMU_BUFFER{IMU_BUFFER_LEN};
std::deque<std::array<double, 6>> IMU_WORLD_BUFFER{IMU_BUFFER_LEN};
std::deque<std::array<double, 3>> VELOCITY_BUFFER{VELOCITY_BUFFER_LEN};
std::deque<std::array<double, 3>> POSITION_BUFFER{POSITION_BUFFER_LEN};
std::deque<std::array<double, 3>> MAG_BUFFER{MAG_BUFFER_LEN};
std::deque<std::array<double, 3>> MAG_WORLD_BUFFER{MAG_BUFFER_LEN};
std::deque<std::array<double, 3>> IPS_BUFFER{IPS_BUFFER_LEN};
std::deque<euler_orientation_t> EULER_ORIENTATION_BUFFER{ORIENTATION_BUFFER_LEN};
std::deque<quaternion> QUAT_ORIENTATION_BUFFER{ORIENTATION_BUFFER_LEN};

// Stance flags.
int STATUS_FLAGS = 0;

// Kill flag for all threads.
int SHUTDOWN = 0;

// Mutex locks.
std::mutex IMU_BUFFER_LOCK;
std::mutex MAG_BUFFER_LOCK;
std::mutex VELOCITY_BUFFER_LOCK;
std::mutex STATUS_FLAG_LOCK;
std::mutex POSITION_BUFFER_LOCK;
std::mutex ORIENTATION_BUFFER_LOCK;

/** \brief Periodically logs status messages to stdout.
 * Useful for debugging or testing, but probably not wanted at runtime.
 * Output is of a form that can be easily piped to other processes.
 */
void std_output()
{
    if (LOG_TO_STDOUT)
    {
        // Output string built here.
        std::stringstream ss;

        // Set up timing, including pause while IMU warms up.
        std::chrono::milliseconds interval(100);
        std::this_thread::sleep_for(interval*3);
        auto time = std::chrono::system_clock::now();

        while (!SHUTDOWN)
        {
            // Add position to the string stream.        
            POSITION_BUFFER_LOCK.lock();
            ss << "Position:        (" << POSITION_BUFFER.back()[0] << ", " << POSITION_BUFFER.back()[1] << ", " << POSITION_BUFFER.back()[2] << ")" << "\n";
            POSITION_BUFFER_LOCK.unlock();

            // Add Euler and quaternion orientations to the string stream.
            ORIENTATION_BUFFER_LOCK.lock();
            ss << "Orientation (E): (" << EULER_ORIENTATION_BUFFER.back().roll << ", " << EULER_ORIENTATION_BUFFER.back().pitch << ", " << EULER_ORIENTATION_BUFFER.back().yaw << ")" << "\n";;
            ss << "Orientation (Q): (" << QUAT_ORIENTATION_BUFFER.back().w << ", " << QUAT_ORIENTATION_BUFFER.back().x << ", " << QUAT_ORIENTATION_BUFFER.back().y << ", " << QUAT_ORIENTATION_BUFFER.back().z << ")" << "\n";
            ORIENTATION_BUFFER_LOCK.unlock();

            // Add stance to the string stream.
            ss << "Stance flag:     " << STATUS.current_stance << "\n";
            ss << "Horizontal:      " << STATUS.attitude << "\n";
            ss << "Fall flag:       " << STATUS.falling << "\n";
            ss << "Entangled flag:  " << STATUS.entangled << "\n";

            // Print the string.
            std::cout << ss.str() << std::endl;

            // Clear the stringstream.
            ss.str("");

            // Wait until next tick.
            time = time + interval;
            std::this_thread::sleep_until(time);
        }
    }
}

/** \brief Reads IMU data, runs orientation filter(s), rotates IMU data, and buffers everything.
 * This is the root of the entire tree. Quite a lot goes on here and a lot of buffers
 * are fed. This is also the most time-sensitive thread. A single loop must not take
 * longer than the reading interval of the IMU, so be careful when implementing
 * any additional functionality here.
 */
void imu_reader()
{
    // Quit immediately if IMU not enabled.
    if (NO_IMU)
    {
        return;
    }

    // Choose an orientation filter depending on configuration, with Madgwick as default.
    arwain::Filter* filter;
    if (CONFIG.orientation_filter == "efaroe")
    {
        filter = new arwain::eFaroe{quaternion{0,0,0,0}, CONFIG.gyro_bias, 100, CONFIG.efaroe_beta, CONFIG.efaroe_zeta};
    }
    else
    {
        filter = new arwain::Madgwick{1000.0/IMU_READING_INTERVAL, CONFIG.madgwick_beta};
    }

    int count = 0;
    int get_mag = 0;

    // Local buffers for IMU data.
    vector3 accel_data;
    vector3 gyro_data;
    vector3 mag_data;
    vector3 world_accel_data;
    vector3 world_gyro_data;
    // vector3 world_mag_data;
    euler_orientation_t euler_data;
    quaternion quat_data;

    // File handles for logging.
    std::ofstream acce_file;
    std::ofstream gyro_file;
    std::ofstream mag_file;
    std::ofstream euler_file;
    std::ofstream quat_file;

    if (LOG_TO_FILE)
    {
        // Open file handles for data logging.
        acce_file.open(FOLDER_DATE_STRING + "/acce.txt");
        gyro_file.open(FOLDER_DATE_STRING + "/gyro.txt");
        mag_file.open(FOLDER_DATE_STRING + "/mag.txt");
        euler_file.open(FOLDER_DATE_STRING + "/euler_orientation.txt");
        quat_file.open(FOLDER_DATE_STRING + "/quat_orientation.txt");

        // File headers
        acce_file << "# time x y z" << "\n";
        gyro_file << "# time x y z" << "\n";
        mag_file << "# time x y z" << "\n";
        euler_file << "# time roll pitch yaw" << "\n";
        quat_file << "# time w x y z" << "\n";
    }

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(5);

    while (!SHUTDOWN)
    {
        // Whether or not to get magnetometer on this spin.
        get_mag = ((count % 20 == 0) && (CONFIG.use_magnetometer || CONFIG.log_magnetometer));

        // Read current sensor values and apply bias correction.
        get_bmi270_data(&accel_data, &gyro_data);
        if (get_mag)
        {
            get_bmm150_data(&mag_data);
            mag_data = mag_data - CONFIG.mag_bias;
            mag_data = mag_data * CONFIG.mag_scale;
        }
        accel_data = accel_data - CONFIG.accel_bias;
        gyro_data = gyro_data - CONFIG.gyro_bias;
        
        // Add new reading to end of buffer, and remove oldest reading from start of buffer.
        IMU_BUFFER_LOCK.lock();
        IMU_BUFFER.pop_front();
        std::array<double, 6> newData{accel_data.x, accel_data.y, accel_data.z, gyro_data.x, gyro_data.y, gyro_data.z};
        if (IMU_BUFFER.back() == newData)
        {
            std::cout << "IMU reading duplication at " << time.time_since_epoch().count() << "\n";
        }
        IMU_BUFFER.push_back(newData);
        IMU_BUFFER_LOCK.unlock();

        // Buffer mag_data if collected. TODO Is there actually any reason to buffer this?
        // if (get_mag)
        // {
        //     MAG_BUFFER_LOCK.lock();
        //     MAG_BUFFER.pop_front();
        //     MAG_BUFFER.push_back(std::array<double, 3>{mag_data.x, mag_data.y, mag_data.z});
        //     MAG_BUFFER_LOCK.unlock();
        // }

        // Log IMU to file.
        if (LOG_TO_FILE)
        {
            acce_file << time.time_since_epoch().count() << " " << accel_data.x << " " << accel_data.y << " " << accel_data.z << "\n";
            gyro_file << time.time_since_epoch().count() << " " << gyro_data.x << " " << gyro_data.y << " " << gyro_data.z << "\n";
            if (CONFIG.log_magnetometer)
            {
                mag_file << time.time_since_epoch().count() << " " << mag_data.x << " " << mag_data.y << " " << mag_data.z << "\n";
            }
        }

        // Perform an orientation filter step.
        if (CONFIG.use_magnetometer)
        {
            filter->update(
                time.time_since_epoch().count(),
                gyro_data.x, gyro_data.y, gyro_data.z,
                accel_data.x, accel_data.y, accel_data.z,
                mag_data.x, mag_data.y, mag_data.z
            );
        }
        else
        {
            filter->update(
                time.time_since_epoch().count(),
                gyro_data.x, gyro_data.y, gyro_data.z,
                accel_data.x, accel_data.y, accel_data.z
            );
        }

        // Extract Euler orientation from filter.
        euler_data.roll = filter->getRoll();
        euler_data.pitch = filter->getPitch();
        euler_data.yaw = filter->getYaw();

        // Extract quaternion orientation from filter.
        quat_data.w = filter->getW();
        quat_data.x = filter->getX();
        quat_data.y = filter->getY();
        quat_data.z = filter->getZ();

        // Add orientation information to buffers.
        ORIENTATION_BUFFER_LOCK.lock();
        EULER_ORIENTATION_BUFFER.pop_front();
        EULER_ORIENTATION_BUFFER.push_back(euler_data);
        QUAT_ORIENTATION_BUFFER.pop_front();
        QUAT_ORIENTATION_BUFFER.push_back(quat_data);
        ORIENTATION_BUFFER_LOCK.unlock();

        // Add world-aligned IMU to its own buffer.
        world_accel_data = world_align(accel_data, quat_data);
        world_gyro_data = world_align(gyro_data, quat_data);
        IMU_BUFFER_LOCK.lock();
        IMU_WORLD_BUFFER.pop_front();
        IMU_WORLD_BUFFER.push_back(std::array<double, 6>{
            world_accel_data.x, world_accel_data.y, world_accel_data.z,
            world_gyro_data.x, world_gyro_data.y, world_gyro_data.z
        });
        IMU_BUFFER_LOCK.unlock();

        // Add world-aligned magnetic field to buffer. TODO Why?
        // if (get_mag)
        // {
        //     world_mag_data = world_align(mag_data, quat_data);
        //     MAG_BUFFER_LOCK.lock();
        //     MAG_WORLD_BUFFER.pop_front();
        //     MAG_WORLD_BUFFER.push_back(std::array<double, 3>{
        //         world_mag_data.x, world_mag_data.y, world_mag_data.z
        //     });
        //     MAG_BUFFER_LOCK.unlock();
        // }

        // Log orientation information to file.
        if (LOG_TO_FILE)
        {
            euler_file << time.time_since_epoch().count() << " " << euler_data.roll << " " << euler_data.pitch << " " << euler_data.yaw << "\n";
            quat_file << time.time_since_epoch().count() << " " << quat_data.w << " " << quat_data.x << " " << quat_data.y << " " << quat_data.z << "\n";
        }
        
        // Wait until the next tick.
        count++;
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    delete filter;

    // Close all file handles.
    if (LOG_TO_FILE)
    {
        acce_file.close();
        gyro_file.close();
        euler_file.close();
        quat_file.close();
        mag_file.close();
    }
}

/** \brief Forms and transmits LoRa messages on a loop.
 */
void transmit_lora()
{
    // Turn on the LoRa radio and quit if it fails.
    int SPI_CHANNEL = 1;
    int CS_PIN = 26;
    int DIO0_PIN = 15;
    int RESET_PIN = 22;
    LoRa lora(SPI_CHANNEL, CS_PIN, DIO0_PIN, RESET_PIN);
    if (!lora.begin())
    {
        std::cout << "LoRa radio failed to start" << std::endl;
        SHUTDOWN = 1;
        return;
    }

    // Configura LoRa radio.
    std::cout << "LoRa radio started successfully" << std::endl;
    lora.setFrequency(CONFIG.lora_rf_frequency);
    lora.setTXPower(23);
    lora.setSpreadFactor(CONFIG.lora_spread_factor);
    lora.setBandwidth(CONFIG.lora_bandwidth);
    lora.setCodingRate(CONFIG.lora_coding_rate);
    lora.setSyncWord(0x12);
    lora.setHeaderMode(CONFIG.lora_header_mode);
    if (CONFIG.lora_enable_crc)
    {
        lora.enableCRC();
    }

    // TEST Check the radio is set up correctly.
    // std::cout << lora.getTXPower() << "\n";
    // printf("%d\n", lora.getTXPower());
    // std::cout << lora.getFrequency() << "\n";
    // std::cout << lora.getSpreadFactor() << "\n";
    // std::cout << lora.getBandwidth() << "\n";
    // std::cout << lora.getCodingRate() + 4 << "\n";
    // // std::cout << lora.getSyncWord() << "\n";
    // printf("%d\n", lora.getSyncWord());
    // std::cout << lora.getHeaderMode() << std::endl;

    // Local buffers.
    std::ofstream lora_file;
    std::array<double, 3> position;
    uint16_t alerts;
    FLOAT16 x16, y16, z16;

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(LORA_TRANSMISSION_INTERVAL);

    // Open file handles for data logging.
    if (LOG_TO_FILE)
    {
        lora_file.open(FOLDER_DATE_STRING + "/lora_log.txt");
        lora_file << "# time packet" << "\n";
    }

    while (!SHUTDOWN)
    {
        // Get positions as float16.
        POSITION_BUFFER_LOCK.lock();
        position = POSITION_BUFFER.back();
        POSITION_BUFFER_LOCK.unlock();
        position[0] = 1.264;
        position[1] = 0.168;
        position[2] = -1.675;
        x16 = (FLOAT16)position[0];
        y16 = (FLOAT16)position[1];
        z16 = (FLOAT16)position[2];

        // Create alerts flags.
        alerts = 0;
        alerts |= STATUS.falling;
        alerts |= (STATUS.entangled << 1) ;
        alerts |= (STATUS.current_stance << 2);

        // Reset critical status flags now they have been read.
        STATUS.falling = arwain::StanceDetector::NotFalling;
        STATUS.entangled = arwain::StanceDetector::NotEntangled;
        
        // TODO Build packet for transmission.
        char message[LORA_MESSAGE_LENGTH];

        // Copy the float16 position values and the alert flags into the buffer.
        memcpy(message, &(x16.m_uiFormat), sizeof(x16.m_uiFormat));
        memcpy(message + 2, &(y16.m_uiFormat), sizeof(x16.m_uiFormat));
        memcpy(message + 4, &(z16.m_uiFormat), sizeof(x16.m_uiFormat));
        memcpy(message + 6, &alerts, sizeof(alerts));

        // This block tests recovery of the float.
        // FLOAT16 f;
        // memcpy(&(f.m_uiFormat), message, 2);
        // float g = FLOAT16::ToFloat32(f);
        // std::cout << g << std::endl;

        // Send transmission.
        LoRaPacket packet{(unsigned char *)message, LORA_MESSAGE_LENGTH};
        lora.transmitPacket(&packet);

        // TODO: Log LoRa transmission to file, including any success/signal criteria that might be available.
        // TODO: Currently logging binary nonsense. Fix.
        if (LOG_TO_FILE)
        {
            lora_file << time.time_since_epoch().count() << " " << message << "\n";
        }
        
        // Wait until next tick
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    // Close log file handle.
    if (LOG_TO_FILE)
    {
        lora_file.close();
    }
}

// TODO Everything from here until the finish line can be removed if we fully convert to vector3 --
// These functions are addition etc. operators for std::array<double, .>, which is being phased out
// in preference for vector3 objects.

// Overload array operator*.
template <class T, class U>
T operator*(const T& a1, U scalar)
{
  T a;
  for (typename T::size_type i = 0; i < a1.size(); i++)
    a[i] = a1[i]*scalar;
  return a;
}

// Overload array operator-.
template <class T>
T operator-(const T& a1, const T& a2)
{
  T a;
  for (typename T::size_type i = 0; i < a1.size(); i++)
    a[i] = a1[i] - a2[i];
  return a;
}

// Overload array operator+.
template <class T>
T operator+(const T& a1, const T& a2)
{
  T a;
  for (typename T::size_type i = 0; i < a1.size(); i++)
    a[i] = a1[i] + a2[i];
  return a;
}

// Finish line ------------------------------------------------------------------------------------

/** \brief Integrates acceleration data over a small window to generate velocity delta.
 * \param data The buffer to integrate over.
 * \param dt The time delta between buffer values.
 * \param offset The position in the buffer entries of the first acceleration value.
 * \return An array of x, y, z velocities.
 */
std::array<double, 3> integrate(std::deque<std::array<double, 6>> &data, double dt, unsigned int offset = 0)
{
    // TODO What the hell was I thinking here? This definitely won't work.
    std::array<double, 3> acc_mean = {-0.182194123, -0.59032666517, 9.86202363151991};
    std::array<double, 3> integrated_data = {0, 0, 0};
    for (unsigned int i = 0; i < data.size(); i++)
    {
        for (unsigned int j = 0; j < 3; j++)
        {
            integrated_data[j] = integrated_data[j] + (data[i][j + offset] - acc_mean[j]) * dt;
        }
    }
    return integrated_data;
}

/** \brief Converts the IMU deque into a C-style array for passing to the inference library.
 * \param[out] data C-style array of shape [1][6][200] in which to store result.
 * \param[in] imu The latest world IMU buffer.
 */
void torch_array_from_deque(float data[1][6][200], std::deque<std::array<double, 6>> imu)
{
    for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 200; k++)
            {
                // Switch the order of gyro and acceleration, and put into array.
                data[i][j][k] = (float)(imu[k][(j+3)%6]);
            }
        }
    }
}

/** \brief This has only been developed to the proof of concept stage and is not suitable for deployment.
 */
void predict_velocity_dispatch()
{
    // *********************************************************************//
    // NOTE This has only been developed to the proof of concept stage.     //
    // It will not work fully without additional work.                      //
    // *********************************************************************//

    // Wait for enough time to ensure the IMU buffer contains valid and useful data before starting.
    std::chrono::milliseconds presleep(1000);
    std::this_thread::sleep_for(presleep*3);

    // Set up socket
    void *context = zmq_ctx_new();
    void *responder = zmq_socket(context, ZMQ_REP);
    zmq_bind(responder, "tcp://*:5555");

    // Buffer to contain local copy of IMU data.
    std::deque<std::array<double, 6>> imu;

    // Request and response string buffers.
    std::stringstream request;
    char answer_buffer[50];

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(VELOCITY_PREDICTION_INTERVAL);

    double x, y;

    while (!SHUTDOWN)
    {
        // Grab latest IMU packet
        IMU_BUFFER_LOCK.lock();
        imu = IMU_WORLD_BUFFER;
        IMU_BUFFER_LOCK.unlock();

        // Load the IMU data into a string for serial transmission.
        for (unsigned int i = 0; i < imu.size(); i++)
        {
            for (unsigned int j = 0; j < 6; j++)
            {
                request << (float)imu[i][j] << ",";
            }
        }

        // Send the data and await response.
        std::string fromStream = request.str();
        const char *str = fromStream.c_str();
        zmq_send(responder, str, strlen(str), 0);
        zmq_recv(responder, answer_buffer, 50, 0);
        request.str("");

        // TODO Process the answer buffer.
        std::string answer{answer_buffer};
        if (answer == "accept")
        {
            continue;
        }
        int delimiter = answer.find(",");
        std::stringstream(answer.substr(0, delimiter)) >> x;
        std::stringstream(answer.substr(delimiter + 1)) >> y;

        std::cout << x << " " << y << "\n";

        // Wait until next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }
}

/** \brief Periodically makes velocity predictions based on data buffers, and adds that velocity and thereby position to the relevant buffers.
 */
void predict_velocity()
{
    // Skip inference if command line says so.
    if (!NO_INFERENCE)
    {
        // TODO: Merge the inference code into this function. Will need further abstraction?
        // TODO: Set up NPU and feed in model.
        // TODO: Make it possible to specify the model file path, and fail gracefully if not found.
        // Torch model{"./xyzronin_v0-5_all2D_small.pt", {1, 6, 200, 1}};
        Torch model{"./xyzronin_v0-6.pt", {1, 6, 200}};

        // Wait for enough time to ensure the IMU buffer contains valid and useful data before starting.
        std::chrono::milliseconds presleep(1000);
        std::this_thread::sleep_for(presleep*3);

        // Set up timing.
        auto time = std::chrono::system_clock::now();
        std::chrono::milliseconds interval(VELOCITY_PREDICTION_INTERVAL);

        // Initialize buffers to contain working values.
        std::array<double, 3> vel;                        // The sum of npu_vel and imu_vel_delta.
        std::array<double, 3> npu_vel;                    // To hold the neural network prediction of velocity.
        std::array<double, 3> vel_previous = {0, 0, 0};   // Contains the velocity calculation from the previous loop.
        std::array<double, 3> npu_vel_delta;              // Stores the difference between the npu velocity prediction and vel_previous.
        std::array<double, 3> imu_vel_delta;              // To integrate the velocity based on IMU readings.
        std::array<double, 3> position;
        std::deque<std::array<double, 6>> imu;            // To contain the last second of IMU data.
        std::deque<std::array<double, 6>> imu_latest;     // To contain the last VELOCITY_PREDICTION_INTERVAL of IMU data.

        // File handles for logging.
        std::ofstream position_file;
        std::ofstream velocity_file;

        // Time in seconds between inferences.
        double interval_seconds = ((double)(VELOCITY_PREDICTION_INTERVAL))/1000.0;

        // TEST How far back to look in the IMU buffer for integration.
        int backtrack = (int)((1000/IMU_READING_INTERVAL)*interval_seconds);

        // Open files for logging.
        if (LOG_TO_FILE)
        {
            velocity_file.open(FOLDER_DATE_STRING + "/velocity.txt");
            position_file.open(FOLDER_DATE_STRING + "/position.txt");
            velocity_file << "# time x y z" << "\n";
            position_file << "# time x y z" << "\n";
        }

        float data[1][6][200];
        
        while (!SHUTDOWN)
        {
            // Grab latest IMU packet
            IMU_BUFFER_LOCK.lock();
            imu = IMU_WORLD_BUFFER;
            IMU_BUFFER_LOCK.unlock();

            // TEST Create data array that torch can understand.
            torch_array_from_deque(data, imu);

            // TEST Make velocity prediction
            std::vector<double> v = model.infer(data);
            npu_vel = {v[0], v[1], v[2]};
            
            // TEST Find the change in velocity from the last period, as predicted by the npu.
            npu_vel_delta = npu_vel - vel_previous;

            // TEST Get last interval worth of IMU data.
            imu_latest = {imu.end() - backtrack, imu.end()};

            // TEST Single integrate the small IMU slice to get delta-v over the period.
            imu_vel_delta = integrate(imu_latest, interval_seconds );

            // TEST Weighted combination of velocity deltas from NPU and IMU integration.
            // TODO This gives very wrong results when trusting the IMU at all. Investigate and repair (or bin it).
            vel = npu_vel_delta*CONFIG.npu_vel_weight_confidence + imu_vel_delta*(1-CONFIG.npu_vel_weight_confidence);

            // TEST Add the filtered delta onto the previous vel estimate and add to buffer.
            vel = vel + vel_previous;

            VELOCITY_BUFFER_LOCK.lock();
            VELOCITY_BUFFER.pop_front();
            VELOCITY_BUFFER.push_back(vel);
            VELOCITY_BUFFER_LOCK.unlock();

            // TEST Store the velocity for use in the next loop (saves having to access the buffer for a single element).
            vel_previous = vel;

            // Iterate velocity onto position to get new position.
            position[0] = position[0] + interval_seconds * vel[0];
            position[1] = position[1] + interval_seconds * vel[1];
            position[2] = position[2] + interval_seconds * vel[2];
            
            // Update position buffer.
            POSITION_BUFFER_LOCK.lock();
            POSITION_BUFFER.pop_front();
            POSITION_BUFFER.push_back(position);
            POSITION_BUFFER_LOCK.unlock();

            // Add position and velocity data to file.
            if (LOG_TO_FILE)
            {
                velocity_file << time.time_since_epoch().count() << " " << vel[0] << " " << vel[1] << " " << vel[2] << "\n";
                position_file << time.time_since_epoch().count() << " " << position[0] << " " << position[1] << " " << position[2] << "\n";
            }

            // Wait until next tick.
            time = time + interval;
            std::this_thread::sleep_until(time);
        }

        // Close file handle(s).
        if (LOG_TO_FILE)
        {
            velocity_file.close();
            position_file.close();
        }
    }
}

/** \brief Capture the SIGINT signal for clean exit.
 * Sets the global SHUTDOWN flag informing all threads to clean up and exit.
 * \param signal The signal to capture.
 */
void sigint_handler(int signal)
{
    if (signal == SIGINT)
    {
        std::cout << "\nReceived SIGINT - closing\n" << "\n";
        SHUTDOWN = 1;
    }
}

/** \brief Indoor positioning system for recognising and snapping to stairs, floors, etc.
 * Runs as thread.
 */
void indoor_positioning()
{
    if (CONFIG.use_indoor_positioning_system)
    {
        // TODO Create IPS object
        arwain::IndoorPositioningWrapper ips;
        std::array<double, 3> velocity;
        std::array<double, 3> position;

        std::ofstream ips_position_file;

        if (LOG_TO_FILE)
        {
            ips_position_file.open(FOLDER_DATE_STRING + "/ips_position.txt");
            ips_position_file << "# time x y z" << "\n";
        }

        // Set up timing.
        auto time = std::chrono::system_clock::now();
        std::chrono::milliseconds interval(INDOOR_POSITIONING_INTERVAL);

        while (!SHUTDOWN)
        {
            // Get most recent velocity data.
            VELOCITY_BUFFER_LOCK.lock();
            velocity = VELOCITY_BUFFER.back();
            VELOCITY_BUFFER_LOCK.unlock();

            // Run update and get new position.
            ips.update(
                time.time_since_epoch().count(),
                velocity[0],
                velocity[1],
                velocity[2]
            );
            position = ips.getPosition();

            // Put IPS position in buffer.
            POSITION_BUFFER_LOCK.lock();
            IPS_BUFFER.pop_front();
            IPS_BUFFER.push_back(position);
            POSITION_BUFFER_LOCK.unlock();

            // Log result to file.
            if (LOG_TO_FILE)
            {
                ips_position_file << time.time_since_epoch().count() << " " << position[0] << " " << position[1] <<  " " << position[2] << "\n";
            }

            // Wait until next tick.
            time = time + interval;
            std::this_thread::sleep_until(time);
        }

        // Close file handle(s);
        if (LOG_TO_FILE)
        {
            ips_position_file.close();
        }
    }
}

/** \brief Stance detection thread, periodically assesses mode of motion based in IMU and velocity data.
 * Runs as a thread.
 */
void stance_detector()
{
    // Stance detector object.
    arwain::StanceDetector stance{
        CONFIG.freefall_sensitivity,
        CONFIG.crawling_threshold,
        CONFIG.running_threshold,
        CONFIG.walking_threshold,
        CONFIG.active_threshold,
        CONFIG.struggle_threshold
    };

    // Local buffers.
    std::deque<std::array<double, 6>> imu_data;
    std::deque<std::array<double, 3>> vel_data;

    // Open file for freefall/entanglement logging
    std::ofstream freefall_file;
    if (LOG_TO_FILE)
    {
        freefall_file.open(FOLDER_DATE_STRING + "/freefall.txt");
        freefall_file << "# time freefall entanglement" << "\n";
    }

    // File handle for stance logging.
    std::ofstream stance_file;
    if (LOG_TO_FILE)
    {
        stance_file.open(FOLDER_DATE_STRING + "/stance.txt");
        stance_file << "# time stance" << "\n";
    }

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{STANCE_DETECTION_INTERVAL};

    while (!SHUTDOWN)
    {
        // Get all relevant data.
        IMU_BUFFER_LOCK.lock();
        imu_data = IMU_BUFFER;
        IMU_BUFFER_LOCK.unlock();

        VELOCITY_BUFFER_LOCK.lock();
        vel_data = VELOCITY_BUFFER;
        VELOCITY_BUFFER_LOCK.unlock();

        // Update stance detector and get output. This can turn on but cannot turn off the falling and entangled flags.
        stance.run(imu_data, vel_data);
        STATUS.current_stance = stance.getStance();
        STATUS.falling = STATUS.falling | stance.getFallingStatus();
        STATUS.entangled = STATUS.entangled | stance.getEntangledStatus();
        STATUS.attitude = stance.getAttitude();

        // Log to file.
        if (LOG_TO_FILE)
        {
            freefall_file << time.time_since_epoch().count() << " " << stance.getFallingStatus() << " " << stance.getEntangledStatus() << "\n";
            stance_file << time.time_since_epoch().count() << " " << stance.getStance() << "\n";
        }

        // Wait until the next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    // Close files
    if (LOG_TO_FILE)
    {
        freefall_file.close();
        stance_file.close();
    }
}

/** \brief Main loop.
 * \param argc Nmber of arguments.
 * \param argv List of arguments.
 */
int main(int argc, char **argv)
{
    // Prepare keyboard interrupt signal handler to enable graceful exit.
    std::signal(SIGINT, sigint_handler);

    // Determine logging behaviour from command line arguments.
    arwain::InputParser input{argc, argv};
    if (input.contains("-h") || input.contains("-help"))
    {
        // Output help text.
        std::cout << "Run without arguments for no logging\n";
        std::cout << "\n";
        std::cout << "Arguments:\n";
        std::cout << "  -lstd        Log friendly output to stdout\n";
        std::cout << "  -lfile       Record sensor data to file - files are stored in ./data_<datetime>\n";
        std::cout << "  -conf        Specify alternate configuration file\n";
        std::cout << "  -testimu     Sends IMU data (a,g) to stdout - other flags are ignored if this is set\n";
        std::cout << "  -noinf       Do not do velocity inference\n";
        std::cout << "  -noimu       Do not turn on the IMU - for testing\n";
        std::cout << "  -h           Show this help text\n";
        std::cout << "\n";
        std::cout << "Example usage:\n";
        std::cout << "  ./arwain -lstd -calib calib.txt -conf arwain.conf -lfile\n";
        std::cout << "\n";
        std::cout << "Error codes:\n";
        std::cout << "   1           Successfully executed\n";
        std::cout << "  -1           IMU failed to start\n";
        std::cout << "  -2           Problem reading configuration file\n";
        return 1;
    }
    if (input.contains("-testimu"))
    {
        arwain::test_imu(SHUTDOWN);
        return 1;
    }
    if (input.contains("-lstd"))
    {
        // Enable stdout logging.
        LOG_TO_STDOUT = 1;
    }
    if (input.contains("-noinf"))
    {
        NO_INFERENCE = 1;
    }
    if (input.contains("-noimu"))
    {
        NO_IMU = 1;
    }
    if (input.contains("-lfile"))
    {
        // Enable file logging.
        std::cout << "Logging to file" << "\n";
        LOG_TO_FILE = 1;
    }
    if (input.contains("-conf"))
    {
        // If alternate configuration file supplied, read it instead of default.
        CONFIG_FILE = input.getCmdOption("-conf");
    }
    if (!input.contains("-lstd") && !input.contains("-lfile"))
    {
        std::cerr << "No logging enabled - you probably want to use -lstd or -lfile or both" << "\n";
    }

    // Attempt to read the config file and quit if failed.
    try
    {
        CONFIG = arwain::get_configuration(CONFIG_FILE);
        if (LOG_TO_STDOUT)
        {
            std::cout << "Configuration file read\n";
        }
    }
    catch (int n)
    {
        std::cout << "Problem reading configuration file\n";
        return -2;
    }
    
    // Create output directory and write copy of current configuration.
    if (LOG_TO_FILE)
    {
        FOLDER_DATE_STRING = "./data_" + arwain::datetimestring();
        if (!std::experimental::filesystem::is_directory(FOLDER_DATE_STRING))
        {
            std::experimental::filesystem::create_directory(FOLDER_DATE_STRING);
        }
        std::experimental::filesystem::copy(CONFIG_FILE, FOLDER_DATE_STRING + "/config.conf");
    }

    // Initialize the IMU.
    if (!NO_IMU)
    {
        if (init_bmi270(CONFIG.use_magnetometer || CONFIG.log_magnetometer, "none") != 0)
        {
            std::cout << "IMU failed to start" << "\n";
            return -1;
        }
    }

    // Start threads.
    std::thread imu_reader_thread(imu_reader);
    std::thread predict_velocity_thread(predict_velocity);
    std::thread stance_detector_thread(stance_detector);
    std::thread transmit_lora_thread(transmit_lora);
    std::thread std_output_thread(std_output);
    std::thread indoor_positioning_thread(indoor_positioning);

    set_thread_priority(imu_reader_thread, 1);

    // Wait for all threads to terminate.
    imu_reader_thread.join();
    predict_velocity_thread.join();
    stance_detector_thread.join();    
    transmit_lora_thread.join();
    std_output_thread.join();
    indoor_positioning_thread.join();

    return 1;
}
