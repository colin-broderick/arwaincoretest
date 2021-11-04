#ifndef _ARWAIN_TIMERS_HPP
#define _ARWAIN_TIMERS_HPP

#include <chrono>
#include <iostream>
#include <string>

namespace arwain::Timers
{
    class ScopedTimer
    {
        public:
            ScopedTimer(const std::string& name) : ScopedTimer(name, std::cout)
            {
            }
            ScopedTimer(const std::string& name, std::ostream& out_stream)
            {
                _name = name;
                _start = std::chrono::high_resolution_clock::now();
                _out_stream = &out_stream;
            }
            ~ScopedTimer()
            {
                _end = std::chrono::high_resolution_clock::now();
                *_out_stream << _name << " took " << (_end - _start).count() << " ns" << std::endl;
            }

        private:
            std::string _name;
            std::ostream* _out_stream;
            std::chrono::system_clock::time_point _start;
            std::chrono::system_clock::time_point _end;
    };

    /** \brief Provides a countdown timer. */
    class CountdownTimer
    {
        private:
            std::chrono::high_resolution_clock::time_point timer_start;
            std::chrono::milliseconds timer_duration;

        public:
            /** \brief Constructor. */
            CountdownTimer(const int timer_duration_ms)
            {
                this->timer_start = std::chrono::high_resolution_clock::now();
                this->timer_duration = std::chrono::milliseconds{timer_duration_ms};
            }

            /** \brief Check whether the timer duration has elapsed. */
            bool finished() const
            {
                return (std::chrono::high_resolution_clock::now() - this->timer_duration > timer_start);
            }

            /** \brief Reset the start time of the timer. */
            void reset()
            {
                this->timer_start = std::chrono::high_resolution_clock::now();
            }
    };
}

#endif
