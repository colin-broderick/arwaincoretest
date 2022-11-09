#ifndef _ARWAIN_TIMERS_HPP
#define _ARWAIN_TIMERS_HPP

#include <chrono>
#include <iostream>
#include <string>

namespace arwain::Timers
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
}

#endif
