#ifndef _ARWAIN_UTILS_HPP
#define _ARWAIN_UTILS_HPP

#include <chrono>
#include <thread>

void sleep_ms(int ms);

class Quaternion;
class Vector3;

struct EulerOrientation
{
    double roll;
    double pitch;
    double yaw;
};

namespace arwain
{
    EulerOrientation compute_euler(Quaternion& quaternion_rotor);
    Vector3 apply_quat_rotor_to_vector3(const Vector3& vector, const Quaternion& quaternion);

    /** \brief Schedules sleeps which end at a specified interval.
     * When the JobInterval is created, the current time is recorded. Each time .await() is called,
     * we increment the stored time by the specified duration, and then do a blocking sleep until
     * that time is reached.
     * \param DurationType A duration type from the chrono header, such as seconds, milliseconds, etc.
     */
    template <class DurationType>
    class JobInterval
    {
        public:
            /** \brief Constructor. Records the current time and the interval to increment by when
             * await is called.
             * \param time_count An integer represent the number of DurationType time units.
             */
            JobInterval(unsigned int time_count)
            : time(std::chrono::high_resolution_clock::now()), interval(DurationType{time_count})
            {

            }

            /** \brief Increments the stored time by the stored interval,
             * then does a blocking sleep until that time is reached.
             * \return Boolean true if a sleep was performed. Boolean false if the target
             * time was in the past and no sleep was needed.
             */
            bool await()
            {
                time = time + interval;
                if (time < std::chrono::high_resolution_clock::now())
                {
                    return false;
                }
                else
                {
                    std::this_thread::sleep_until(time);
                    return true;
                }
            }

        private:
            std::chrono::high_resolution_clock::time_point time;
            DurationType interval;
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
