#ifndef _ARWAIN_TIMERS_HPP
#define _ARWAIN_TIMERS_HPP

#include <chrono>
#include <iostream>
#include <string>
#include <thread>

namespace Timers
{
    /** \brief Implements a scoped timer; that is, a timer which reports how
     * how long it existed for when the destructor is called.
     */
    class ScopedTimer
    {
        public:
            ScopedTimer(const std::string& name);
            ScopedTimer(const std::string& name, std::ostream& out_stream);
            ~ScopedTimer();
        private:
            std::string _name;
            std::ostream* _out_stream;
            std::chrono::high_resolution_clock::time_point _start;
            std::chrono::high_resolution_clock::time_point _end;
    };
    
    /** \brief Implements a polled countdown timer; finished() will return true
     * after the given time has elapsed.
     */
    class CountdownTimer
    {
        public:
            CountdownTimer(const int timer_duration_ms);
            bool finished() const;
            void reset();
        private:
            std::chrono::high_resolution_clock::time_point timer_start;
            std::chrono::milliseconds timer_duration;
    };

    /** \brief Schedules sleeps which end at a specified interval.
     * When the JobInterval is created, the current time is recorded. Each time .await() is called,
     * we increment the stored time by the specified duration, and then do a blocking sleep until
     * that time is reached.
     * \param DurationType A duration type from the chrono header, such as seconds, milliseconds, etc.
     */
    template <class DurationType>
    class IntervalTimer
    {
        public:
            /** \brief Constructor. Records the current time and the interval to increment by when
             * await is called.
             * \param time_count An integer represent the number of DurationType time units.
             */
            IntervalTimer(unsigned int time_count)
            : time(std::chrono::high_resolution_clock::now()), interval(DurationType{time_count})
            {

            }

            IntervalTimer(unsigned int time_count, const std::string& name)
            : IntervalTimer(time_count)
            {
                stored_name = name;
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
                    std::cout << "WARNING: Interval timer named \"" << stored_name << "\" appears to be running slow\n";
                    return false;
                }
                else
                {
                    std::cout << "Sleeping\n";
                    std::this_thread::sleep_until(time);
                    return true;
                }
            }

            /** \brief
             * 
            */
            int64_t count()
            {
                return time.time_since_epoch().count();
            }

        TESTABLE:
            std::chrono::high_resolution_clock::time_point time;
            DurationType interval;
            std::string stored_name = "no_name_set";
    };
}

#endif
