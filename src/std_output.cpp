#include <sstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <deque>

#include "arwain.hpp"
#include "vector3.hpp"
#include "quaternion.hpp"
#include "filter.hpp"

/** \brief Periodically logs status messages to stdout.
 * Useful for debugging or testing, but probably not wanted at runtime.
 * Output is of a form that can be easily piped to other processes.
 */
void std_output()
{
    if (arwain::config.log_to_stdout)
    {
        // Output string built here.
        std::stringstream ss;

        // Set up timing, including pause while IMU warms up.
        std::chrono::milliseconds interval{arwain::Intervals::STD_OUT_INTERVAL};
        std::this_thread::sleep_for(interval*3);
        auto time = std::chrono::system_clock::now();

        // For magnetic orientation
        vector3 mag_target{18.895, -0.361, 45.372};  // Local magnetic vector.
        mag_target = normalised(mag_target);
        vector3 mag_data = mag_target;               // Measured magnetic field.
        vector3 new_mag_data{0, 0, 0};
        double angle;
        vector3 axis;
        quaternion quat2;


        while (!arwain::shutdown)
        {
            { // Add position to the string stream.
                std::lock_guard<std::mutex> lock{arwain::Locks::POSITION_BUFFER_LOCK};
                ss << "Position:        " << arwain::Buffers::POSITION_BUFFER.back() << "\n";
            }

            // Add Euler and quaternion orientations to the string stream.
            quaternion quat;
            {
                std::lock_guard<std::mutex> lock{arwain::Locks::ORIENTATION_BUFFER_LOCK};
                quat = arwain::Buffers::QUAT_ORIENTATION_BUFFER.back();
            }
            auto euler_angles = arwain::Filter::getEulerAnglesDegrees(quat.w, quat.x, quat.y, quat.z);
            ss << "Orientation (E): " << "R:" << euler_angles[0] << ", " << "P:" << euler_angles[1] << ", " << "Y:" << euler_angles[2] << "\n";;
            ss << "Orientation (Q): " << quat << "\n";

            // Add stance to the string stream.
            ss << "Stance flag:     " << arwain::status.current_stance << "\n";
            ss << "Horizontal:      " << arwain::status.attitude << "\n";
            ss << "Fall flag:       " << arwain::status.falling << "\n";
            ss << "Entangled flag:  " << arwain::status.entangled << "\n";

            if (!arwain::config.no_pressure)
            {
                std::lock_guard<std::mutex> lock{arwain::Locks::PRESSURE_BUFFER_LOCK};
                ss << "Air pressure:    " << arwain::Buffers::PRESSURE_BUFFER.back() << "\n";
            }

            if (1)
            {
                // The following is a magnetic orientation experiment.
                /*
                I am attempting to make a magneto-only orientation filter, i.e. a filter which tells how a sensor is rotated relative to
                the local geomagnetic vector.

                mag_target is the vector representing the expected magnetic field at a given location.
                mag_data is the actual magnetic vector measured by the sensor.
                We compute the angle between these two vectors using the dot product.
                We compute an axis orthogonal to both of these vectors using the cross product.
                Rotating by the computed angle, aroud the orthogonal vector, should carry one of these vectors onto the other.
                We construct a quaternion using the axis-angle pair.
                This quaternion describes how rotated our sensor is compared to the local geomagnetic vector.
                */
                {
                    std::lock_guard<std::mutex> lock{arwain::Locks::MAG_BUFFER_LOCK};
                    new_mag_data = arwain::Buffers::MAG_BUFFER.back();
                }
                new_mag_data = normalised(new_mag_data);           // Normalize the measured magnetic field.
                mag_data.x = 0.95 * mag_data.x + 0.05 * new_mag_data.x;
                mag_data.y = 0.95 * mag_data.y + 0.05 * new_mag_data.y;
                mag_data.z = 0.95 * mag_data.z + 0.05 * new_mag_data.z;

                mag_data = normalised(mag_data);

                angle = acos(mag_data.x*mag_target.x 
                           + mag_data.y*mag_target.y
                           + mag_data.z*mag_target.z);           // Angle between measured field and expected local field.
                axis = cross(mag_data, mag_target);                // An axis orthogonal to the local field vector and measured field vector.
                quat2 = quaternion{                                // Quaternion representaiton of how device is rotated relative to local field.
                    cos(angle/2.0),
                    sin(angle/2.0) * axis.x,
                    sin(angle/2.0) * axis.y,
                    sin(angle/2.0) * axis.z
                };
                ss << "Magnetic vector: " << quat2 << "\n";
            }

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
