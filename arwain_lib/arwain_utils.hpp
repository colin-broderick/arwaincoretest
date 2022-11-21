#ifndef _ARWAIN_UTILS_HPP
#define _ARWAIN_UTILS_HPP

#include <chrono>
#include <thread>
#include <deque>
#include <map>

void sleep_ms(int ms);

class Quaternion;
class Vector3;

double unwrap_phase_degrees(double new_angle, double previous_angle);
double unwrap_phase_radians(double new_angle, double previous_angle);

struct EulerOrientation
{
    double roll;
    double pitch;
    double yaw;
};

/** \brief Computes a true rolling average as values are fed in.
 * 
 * Averages will be produced and can be obtained before the window is filled,
 * although this will likely not be a useful value before the averaging window
 * is filled. The method .ready() can be called to confirm that the roller has
 * been fed enough values to fill the window and the average value is therefore
 * valid.
 */
class RollingAverage
{
    public:
        RollingAverage() = default;
        RollingAverage(unsigned int window_size_);
        bool ready();
        void feed(double value);
        double get_value();

    private:
        unsigned int window_size;
        double current_average = 0;
        std::deque<double> stack;
};

namespace arwain
{
    EulerOrientation compute_euler(Quaternion& quaternion_rotor);
    Vector3 apply_quat_rotor_to_vector3(const Vector3& vector, const Quaternion& quaternion);
    

    enum class ReturnCode
    {
        Success = 0,
        FailedIMU = -1,
        FailedConfiguration = -2,
        FailedMagnetometer = -3,
        NoConfigurationFile = -4,
        NoInferenceXML = -5,
        IMUReadError = -6,
        IOError = -7
    };

    inline std::map<arwain::ReturnCode, std::string> ErrorMessages = {
        {arwain::ReturnCode::Success, "Success."},
        {arwain::ReturnCode::FailedIMU, "Could not communicate with IMU."},
        {arwain::ReturnCode::FailedConfiguration, "Configuration failed."},
        {arwain::ReturnCode::FailedMagnetometer, "Could not communicate with magnetometer."},
        {arwain::ReturnCode::NoConfigurationFile, "Could not find configuration file."},
        {arwain::ReturnCode::NoInferenceXML, "Inference model XML file not found."},
        {arwain::ReturnCode::IMUReadError, "Encountered error reading IMU."},
        {arwain::ReturnCode::IOError, "Problem with file access."}
    };
    
    /** \brief Compute the average value of an indeterminiate number of objects. */
    template <class T, class... Args> T average(T first, Args... args)
    {
        double count = 1.0;
        for (const auto& arg : {args...})
        {
            first = first + arg;
            count += 1.0;
        }
        return first / count;
    }
}

#endif
