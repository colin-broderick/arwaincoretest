#ifndef _ARWAIN_HPP
#define _ARWAIN_HPP

#include <map>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>

#include <arwain/input_parser.hpp>
#include <arwain/quaternion.hpp>
#include <arwain/devices/rfm95w.hpp>

#if USE_UUBLA
#include <uubla/uubla.hpp>
#endif

#include "arwain/utils.hpp"
#include "arwain/stance.hpp"
#include "arwain/configuration.hpp"
#include "arwain/global_buffer.hpp"
#include "arwain/activity_metric.hpp"
#include "arwain/modes.hpp"
#include "arwain/timers.hpp"

class StanceDetection;

namespace arwain
{
    enum class ReturnCode;
    class InputParser;
}

arwain::ReturnCode arwain_main(int argc, char** argv);

class Vector3;
struct ImuData;

namespace arwain::BufferSizes
{
    inline const unsigned int POSITION_BUFFER_LEN = 200;
    inline const unsigned int MAG_BUFFER_LEN = 200;
    inline const unsigned int VELOCITY_BUFFER_LEN = 200;
    inline const unsigned int ORIENTATION_BUFFER_LEN = 200;
    inline const unsigned int IMU_BUFFER_LEN = 200;
    inline const unsigned int IPS_BUFFER_LEN = 50;
    inline const unsigned int LORA_MESSAGE_LENGTH = 11;
    inline const unsigned int PRESSURE_BUFFER_LEN = 100;
    inline const unsigned int MAG_ORIENTATION_BUFFER_LEN = 100;
    inline const unsigned int MAG_EULER_BUFFER_LEN = 100;
    
    #if USE_UUBLA
    inline const unsigned int LORA_BEACON_MESSAGE_LENGTH = 2;
    #endif
}

namespace arwain
{
    class Logger;
}

namespace arwain::Intervals
{
    // Time intervals, all in milliseconds.
    inline constexpr auto IMU_READING_INTERVAL = 5_ms;
    inline constexpr auto VELOCITY_PREDICTION_INTERVAL = 50_ms;
    inline constexpr auto LORA_TRANSMISSION_INTERVAL = 1000_ms;
    inline constexpr auto STANCE_DETECTION_INTERVAL = 1000_ms;
    inline constexpr auto INDOOR_POSITIONING_INTERVAL = 50_ms;
    inline constexpr auto STD_OUT_INTERVAL = 1000_ms;
    inline constexpr auto IPS_INTERVAL = 500_ms;
    inline constexpr auto MAG_READ_INTERVAL = 100_ms;
    inline constexpr auto ALTIMETER_INTERVAL = 20_ms;
}

namespace arwain
{
    extern double yaw_offset;
    extern std::string folder_date_string;
    extern std::string folder_date_string_suffix;
    extern arwain::Configuration config;
    extern arwain::Logger error_log;
    extern unsigned int velocity_inference_rate;
    extern RollingAverage rolling_average_accel_z_for_altimeter;
    extern ActivityMetric activity_metric;

    void setup_log_folder_name_suffix(const arwain::InputParser& input);
    arwain::ReturnCode execute_jobs();
    arwain::ReturnCode rerun_orientation_filter(const std::string& data_location);
    arwain::ReturnCode rerun_floor_tracker(const std::string& data_location);
    void setup_log_directory();
    arwain::ReturnCode calibrate_gyroscopes_offline();
    arwain::ReturnCode calibrate_accelerometers_simple();
    arwain::ReturnCode calibrate_magnetometers();

    const std::string help_text = "Run without arguments for no logging\n"
        "\n"
        "Arguments:\n"
        "    --lstd          Log friendly output to stdout\n"
        "    --conf          Specify alternate configuration file\n"
        "    --calib         Perform online calibration - make sure the device is totally stationary\n"
        "    --noinf         Do not do velocity inference\n"
        "    --noimu         Do not turn on the IMU - for testing\n"
        "    --nolora        Do not attempt to enable LoRa chip or send transmissions\n"
        "    --nopressure    Do not use the pressure sensor to assist altitude tracking\n"
        "    -h             Show this help text\n"
        "  (The following arguments are exclusive)\n"
        "    --testimu       Sends IMU data (a,g) to stdout - other flags are ignored if this is set\n"
        "    --testori       Runs an orientation filter against IMU 1 and prints results as quaternion and euler triplet\n"
        "    --calibg        Calibrate the gyroscope for a give IMU. Must specify --bus and --address.\n"
        "    --caliba        Calibrate the acceleration for a given IMU. Must specify --bus and --address.\n"
        "                   If a configuration file is specified, the result of the calibrations will be written there.\n"
        "        --bus       The bus on which to find the IMU, e.g. /dev/i2c-1\n"
        "        --address   The address of the IMU in hexadecimal, e.g. 0x68\n"
        "\n"
        "Example usages:\n"
        "    ./arwain --lstd --calib calib.txt --conf arwain.conf\n"
        "    ./arwain --calibg --bus /dev/i2c-1 --address 0x68 --conf /etc/arwain.conf\n"
        "\n"
        "Return codes:\n"
        "     1             Successfully executed\n"
        "    -1             IMU failed to start\n"
        "    -2             Problem reading configuration file\n"
        "    -3             Inference model not found";
}

namespace arwain::Buffers
{
    extern GlobalBuffer<ImuData, arwain::BufferSizes::IMU_BUFFER_LEN> IMU_BUFFER;
    extern GlobalBuffer<ImuData, arwain::BufferSizes::IMU_BUFFER_LEN> IMU_WORLD_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::VELOCITY_BUFFER_LEN> VELOCITY_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::POSITION_BUFFER_LEN> POSITION_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::MAG_BUFFER_LEN> MAG_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::MAG_BUFFER_LEN> MAG_WORLD_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::IPS_BUFFER_LEN> IPS_BUFFER;
    extern GlobalBuffer<Vector3, arwain::BufferSizes::PRESSURE_BUFFER_LEN> PRESSURE_BUFFER;
    extern GlobalBuffer<EulerOrientation, arwain::BufferSizes::ORIENTATION_BUFFER_LEN> EULER_ORIENTATION_BUFFER;
    extern GlobalBuffer<Quaternion, arwain::BufferSizes::ORIENTATION_BUFFER_LEN> QUAT_ORIENTATION_BUFFER;
    extern GlobalBuffer<Quaternion, arwain::BufferSizes::MAG_ORIENTATION_BUFFER_LEN> MAG_ORIENTATION_BUFFER;
    extern GlobalBuffer<double, arwain::BufferSizes::MAG_EULER_BUFFER_LEN> MAG_EULER_BUFFER;
}

#endif
