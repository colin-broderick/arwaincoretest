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

#define THREAD_AFFINITY_EXPERIMENT 0
#define USE_SOCKET_INFERENCE 1
#define USE_SOCKET_RADIO 1
#define GYRO_BIAS_EXPERIMENT 0

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
#include <iomanip>

//#include "arwain_torch.h"
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

#include "velocity_prediction.h"
#include "lora.h"
#include "packet.h"
#include "half.h"
// #include "rknn_api.h"

// Time intervals, all in milliseconds.
unsigned int IMU_READING_INTERVAL = 5;
unsigned int VELOCITY_PREDICTION_INTERVAL = 50;
unsigned int LORA_TRANSMISSION_INTERVAL = 500;
unsigned int STANCE_DETECTION_INTERVAL = 1000;
unsigned int INDOOR_POSITIONING_INTERVAL = 50;

// Buffer sizes.
unsigned int POSITION_BUFFER_LEN = 200;
unsigned int MAG_BUFFER_LEN = 200;
unsigned int VELOCITY_BUFFER_LEN = 200;
unsigned int ORIENTATION_BUFFER_LEN = 200;
unsigned int IMU_BUFFER_LEN = 200;
unsigned int IPS_BUFFER_LEN = 50;
unsigned int LORA_MESSAGE_LENGTH = 8;

// Globally accessible configuration and status.
arwain::Configuration CONFIG;
arwain::Status STATUS;

// std::string inference_tcp_socket = "tcp://*:5555";
std::string radio_tcp_socket = "tcp://*:5556";

// Default config file locations.
std::string CONFIG_FILE = "./arwain.conf";

// Flags for whether to produce various log outputs.
int LOG_TO_STDOUT = 0;
int LOG_TO_FILE = 0;
int NO_INFERENCE = 0;
int NO_IMU = 0;
int NO_LORA = 0;

// Name for data folder
std::string FOLDER_DATE_STRING;

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

#if GYRO_BIAS_EXPERIMENT
void gyro_bias_estimation()
{
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{1057};

    std::deque<std::array<double, 6>> imu;
    std::array<double, 3> gyro_delta_sum;
    std::array<double, 3> gyro_sum;
    std::array<double, 3> bias_estimates = {
        CONFIG.gyro_bias.x,
        CONFIG.gyro_bias.y,
        CONFIG.gyro_bias.z
    };

    while (!SHUTDOWN)
    {
        { // Get IMU data
            std::lock_guard<std::mutex> lock{IMU_BUFFER_LOCK};
            imu = IMU_BUFFER;
        }

        gyro_sum = {0, 0, 0};
        for (int i = 0; i < imu.size(); i++)
        {
            gyro_sum[0] += imu[i][3];
            gyro_sum[1] += imu[i][4];
            gyro_sum[2] += imu[i][5];
        }
        gyro_delta_sum = {0, 0, 0};
        for (int i = 1; i < imu.size(); i++)
        {
            gyro_delta_sum[0] += (imu[i][3] - imu[i-1][3]);
            gyro_delta_sum[1] += (imu[i][4] - imu[i-1][4]);
            gyro_delta_sum[2] += (imu[i][5] - imu[i-1][5]);
        }

        bias_estimates[0] = bias_estimates[0]*0.95 + 0.05*(gyro_sum[0] - gyro_delta_sum[0])/(float)imu.size();
        bias_estimates[1] = bias_estimates[1]*0.95 + 0.05*(gyro_sum[1] - gyro_delta_sum[1])/(float)imu.size();
        bias_estimates[2] = bias_estimates[2]*0.95 + 0.05*(gyro_sum[2] - gyro_delta_sum[2])/(float)imu.size();

        std::cout << bias_estimates[0] << "," << bias_estimates[1] << "," << bias_estimates[2] << std::endl;

        time = time + interval;
        std::this_thread::sleep_until(time);
    }
}
#endif

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
            { // Add position to the string stream.        
                std::lock_guard<std::mutex> lock{POSITION_BUFFER_LOCK};
                ss << "Position:        (" << POSITION_BUFFER.back()[0] << ", " << POSITION_BUFFER.back()[1] << ", " << POSITION_BUFFER.back()[2] << ")" << "\n";
            }

            { // Add Euler and quaternion orientations to the string stream.
                std::lock_guard<std::mutex> lock{ORIENTATION_BUFFER_LOCK};
                ss << "Orientation (E): (" << EULER_ORIENTATION_BUFFER.back().roll << ", " << EULER_ORIENTATION_BUFFER.back().pitch << ", " << EULER_ORIENTATION_BUFFER.back().yaw << ")" << "\n";;
                ss << "Orientation (Q): (" << QUAT_ORIENTATION_BUFFER.back().w << ", " << QUAT_ORIENTATION_BUFFER.back().x << ", " << QUAT_ORIENTATION_BUFFER.back().y << ", " << QUAT_ORIENTATION_BUFFER.back().z << ")" << "\n";
            }

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
        
        { // Add new reading to end of buffer, and remove oldest reading from start of buffer.
            std::lock_guard<std::mutex> lock{IMU_BUFFER_LOCK};
            IMU_BUFFER.pop_front();
            std::array<double, 6> newData{accel_data.x, accel_data.y, accel_data.z, gyro_data.x, gyro_data.y, gyro_data.z};
            if (IMU_BUFFER.back() == newData)
            {
                std::cout << "IMU reading duplication at " << time.time_since_epoch().count() << "\n";
            }
            IMU_BUFFER.push_back(newData);
        }

        // Buffer mag_data if collected. TODO Is there actually any reason to buffer this?
        // if (get_mag)
        // {
        //     std::lock_guard<std::mutex> lock{MAG_BUFFER_LOCK};
        //     MAG_BUFFER.pop_front();
        //     MAG_BUFFER.push_back(std::array<double, 3>{mag_data.x, mag_data.y, mag_data.z});
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

        { // Add orientation information to buffers.
            std::lock_guard<std::mutex> lock{ORIENTATION_BUFFER_LOCK};
            EULER_ORIENTATION_BUFFER.pop_front();
            EULER_ORIENTATION_BUFFER.push_back(euler_data);
            QUAT_ORIENTATION_BUFFER.pop_front();
            QUAT_ORIENTATION_BUFFER.push_back(quat_data);
        }

        // Add world-aligned IMU to its own buffer.
        world_accel_data = world_align(accel_data, quat_data);
        world_gyro_data = world_align(gyro_data, quat_data);
        {
            std::lock_guard<std::mutex> lock{IMU_BUFFER_LOCK};
            IMU_WORLD_BUFFER.pop_front();
            IMU_WORLD_BUFFER.push_back(std::array<double, 6>{
                world_accel_data.x, world_accel_data.y, world_accel_data.z,
                world_gyro_data.x, world_gyro_data.y, world_gyro_data.z
            });
        }

        // Add world-aligned magnetic field to buffer. TODO Why?
        // if (get_mag)
        // {
        //     world_mag_data = world_align(mag_data, quat_data);
        //     std::lock_guard<std::mutex> lock{MAG_BUFFER_LOCK};
        //     MAG_WORLD_BUFFER.pop_front();
        //     MAG_WORLD_BUFFER.push_back(std::array<double, 3>{
        //         world_mag_data.x, world_mag_data.y, world_mag_data.z
        //     });
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

#if USE_SOCKET_INFERENCE
void transmit_lora()
{
    if (NO_LORA)
    {
        return;
    }

    char response_buffer[50];
    // Wait until there's something worth transmitting.
    std::this_thread::sleep_for(std::chrono::milliseconds{3000});

    // Set up log file.
    std::ofstream lora_file;
    if (LOG_TO_FILE)
    {
        lora_file.open(FOLDER_DATE_STRING + "/lora_log.txt");
        lora_file << "# time packet" << "\n";
    }

    // Set up socket.
    void *context = zmq_ctx_new();
    void *responder = zmq_socket(context, ZMQ_REP);
    zmq_bind(responder, radio_tcp_socket.c_str());

    // Local buffers.
    std::array<double, 3> position{0, 0, 0};
    std::stringstream ss;

    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{LORA_TRANSMISSION_INTERVAL};

    while (!SHUTDOWN)
    {
        { // Grab all relevant data from buffers.
            std::lock_guard<std::mutex> lock{POSITION_BUFFER_LOCK};
            position = POSITION_BUFFER.back();
        }

        ss << position[0] << "," << position[1] << "," << position[2] << ",";
        ss << (STATUS.falling == arwain::StanceDetector::Falling ? "f" : "0");
        ss << ",";
        ss << (STATUS.entangled == arwain::StanceDetector::Entangled ? "e" : "0");
        ss << ",";

        STATUS.falling = arwain::StanceDetector::NotFalling;
        STATUS.entangled = arwain::StanceDetector::NotEntangled;

        switch (STATUS.current_stance)
        {
            case arwain::StanceDetector::Inactive:
                ss << "inactive";
                break;
            case arwain::StanceDetector::Walking:
                ss << "walking";
                break;
            case arwain::StanceDetector::Searching:
                ss << "searching";
                break;
            case arwain::StanceDetector::Crawling:
                ss << "crawling";
                break;
            case arwain::StanceDetector::Running:
                ss << "running";
                break;
            default:
                ss << "unknown";
                break;
        }

        // TODO Send by socket and reset stringstream.
        std::string fromStream = ss.str();
        const char* str = fromStream.c_str();
        zmq_send(responder, str, strlen(str), 0);
        zmq_recv(responder, response_buffer, 50, 0);  // Don't need the response.
        ss.str("");

        if (LOG_TO_FILE)
        {
            lora_file << time.time_since_epoch().count() << " " << ss.str() << "\n";
        }

        time = time + interval;
        std::this_thread::sleep_until(time);
    }

    // Instruct transmitter script to close and close log files.
    zmq_send(responder, "stop", strlen("stop"), 0);
    if (LOG_TO_FILE)
    {
        lora_file.close();
    }
}

#else

/** \brief Forms and transmits LoRa messages on a loop.
 */
void transmit_lora()
{
    if (NO_LORA)
    {
        return;
    }

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
    else
    {
        std::cout << "LoRa radio started successfully" << std::endl;
    }

    // Configura LoRa radio.
    lora.setFrequency(CONFIG.lora_rf_frequency);
    lora.setTXPower(CONFIG.lora_tx_power);
    lora.setSpreadFactor(CONFIG.lora_spread_factor);
    lora.setBandwidth(CONFIG.lora_bandwidth);
    lora.setCodingRate(CONFIG.lora_coding_rate);
    lora.setHeaderMode(CONFIG.lora_header_mode);
    // lora.setSyncWord(0x12);
    if (CONFIG.lora_enable_crc)
    {
        // lora.enableCRC();
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
    // FLOAT16 x16, y16, z16;

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval(LORA_TRANSMISSION_INTERVAL);

    // Open file handles for data logging.
    if (LOG_TO_FILE)
    {
        lora_file.open(FOLDER_DATE_STRING + "/lora_log.txt");
        lora_file << "# time packet" << "\n";
    }

    int xa = 0;

    uint64_t testval = 12345678910;
    while (!SHUTDOWN)
    {
        { // Get positions as float16.
            std::lock_guard<std::mutex> lock{POSITION_BUFFER_LOCK};
            position = POSITION_BUFFER.back();
        }
        // std::cout << xa << "\n";
        position[0] = xa;
        position[1] = 3.5;
        position[2] = 1.5;
        FLOAT16 x16{position[0]};
        FLOAT16 y16{position[1]};
        FLOAT16 z16{position[2]};
        xa++;
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


        memcpy(message, &testval, sizeof(uint64_t));
        testval += 1;
        // Copy the float16 position values and the alert flags into the buffer.
        // memcpy(message, &(x16.m_uiFormat), sizeof(x16.m_uiFormat));
        // memcpy(message + 2, &(y16.m_uiFormat), sizeof(x16.m_uiFormat));
        // memcpy(message + 4, &(z16.m_uiFormat), sizeof(x16.m_uiFormat));
        // memcpy(message + 6, &alerts, sizeof(alerts));

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
#endif

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
            { // Get most recent velocity data.
                std::lock_guard<std::mutex> lock{VELOCITY_BUFFER_LOCK};
                velocity = VELOCITY_BUFFER.back();
            }

            // Run update and get new position.
            ips.update(
                time.time_since_epoch().count(),
                velocity[0],
                velocity[1],
                velocity[2]
            );
            position = ips.getPosition();

            { // Put IPS position in buffer.
                std::lock_guard<std::mutex> lock{POSITION_BUFFER_LOCK};
                IPS_BUFFER.pop_front();
                IPS_BUFFER.push_back(position);
            }

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
        { // Get all relevant data.
            std::lock_guard<std::mutex> lock{IMU_BUFFER_LOCK};
            imu_data = IMU_BUFFER;
        }
        {
            std::lock_guard<std::mutex> lock{VELOCITY_BUFFER_LOCK};
            vel_data = VELOCITY_BUFFER;
        }

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

void py_transmitter()
{
    if (NO_LORA)
    {
        return;
    }
    system("python3 ./python_utils/lora_transmitter.py > /dev/null &");
}

void py_inference()
{
    system("python3 ./python_utils/ncs2_interface.py > /dev/null &");
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
    { // Output help text.
        std::cout << "Run without arguments for no logging\n";
        std::cout << "\n";
        std::cout << "Arguments:\n";
        std::cout << "  -lstd        Log friendly output to stdout\n";
        std::cout << "  -lfile       Record sensor data to file - files are stored in ./data_<datetime>\n";
        std::cout << "  -conf        Specify alternate configuration file\n";
        std::cout << "  -testimu     Sends IMU data (a,g) to stdout - other flags are ignored if this is set\n";
        std::cout << "  -noinf       Do not do velocity inference\n";
        std::cout << "  -noimu       Do not turn on the IMU - for testing\n";
        std::cout << "  -nolora      Do not attempt to enable LoRa chip or send transmissions\n";
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
    { // Enable stdout logging.
        LOG_TO_STDOUT = 1;
    }
    if (input.contains("-noinf"))
    {
        NO_INFERENCE = 1;
    }
    if (input.contains("-nolora"))
    {
        NO_LORA = 1;
    }
    if (input.contains("-noimu"))
    {
        NO_IMU = 1;
    }
    if (input.contains("-lfile"))
    { // Enable file logging.
        std::cout << "Logging to file" << "\n";
        LOG_TO_FILE = 1;
    }
    if (input.contains("-conf"))
    { // If alternate configuration file supplied, read it instead of default.
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
    
    // Start external python scripts.
    std::thread py_transmitter_thread{py_transmitter};
    std::thread py_inference_thread{py_inference};
    
    #if GYRO_BIAS_EXPERIMENT
    std::thread gyro_bias_estimator(gyro_bias_estimation);
    #endif

    #if THREAD_AFFINITY_EXPERIMENT
    // Set the IMU thread to run on CPU0 only.
    int rc;
    cpu_set_t imu_cpu_set;
    CPU_ZERO(&imu_cpu_set);
    CPU_SET(0, &imu_cpu_set);
    rc = pthread_setaffinity_np(imu_reader_thread.native_handle(), sizeof(cpu_set_t), &imu_cpu_set);
    if (rc != 0)
    {
        std::cout << "Failed to set thread affinity for IMU\n";
    }

    // Set the inference thread to run on CPUs except that used by the IMU.
    cpu_set_t inference_cpu_set;
    CPU_ZERO(&inference_cpu_set);
    std::cout << "CPUs: " << std::thread::hardware_concurrency() << "\n";
    for (unsigned int i = 1; i < std::thread::hardware_concurrency(); i++)
    {
        CPU_SET(i, &inference_cpu_set);
    }
    rc = pthread_setaffinity_np(predict_velocity_thread.native_handle(), sizeof(cpu_set_t), &inference_cpu_set);
    if (rc != 0)
    {
        std::cout << "Failed to set thread affinity for inference\n";
    }
    #endif

    // Wait for all threads to terminate.
    imu_reader_thread.join();
    predict_velocity_thread.join();
    stance_detector_thread.join();    
    transmit_lora_thread.join();
    std_output_thread.join();
    indoor_positioning_thread.join();
    py_transmitter_thread.join();
    py_inference_thread.join();
    #if GYRO_BIAS_EXPERIMENT
    gyro_bias_estimator.join();
    #endif

    return 1;
}
