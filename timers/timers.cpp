#include "timers.hpp"

/** \brief Create a ScopedTimer with the given name, which reports its result to cout.
 * \param name Readable string for identification purposes.
 */
arwain::Timers::ScopedTimer::ScopedTimer(const std::string& name) : ScopedTimer(name, std::cout)
{
}

/** \brief Create a ScopedTimer with the given name, which reports its result to the provided iostream.
 * \param name Readable string for identification purposes.
 * \param out_stream The iostream to which results should be sent.
*/
arwain::Timers::ScopedTimer::ScopedTimer(const std::string& name, std::ostream& out_stream)
{
    _name = name;
    _start = std::chrono::high_resolution_clock::now();
    _out_stream = &out_stream;
}

/** \brief Report the time for which the ScopedTimer existed, then destory timer. */
arwain::Timers::ScopedTimer::~ScopedTimer()
{
    _end = std::chrono::high_resolution_clock::now();
    *_out_stream << _name << " took " << (_end - _start).count() << " ns" << std::endl;
}

/** \brief Initiate a countdown timer with the given duration in milliseconds.
 * \param timer_duration_ms How long should elapse before the countdown is considered complete.
 */
arwain::Timers::CountdownTimer::CountdownTimer(const int timer_duration_ms)
{
    this->timer_start = std::chrono::high_resolution_clock::now();
    this->timer_duration = std::chrono::milliseconds{timer_duration_ms};
}

/** \brief Check whether the timer duration has elapsed.
 * \return Boolean flag indicate whether the countdown has finished or not.
 */
bool arwain::Timers::CountdownTimer::finished() const
{
    return (std::chrono::high_resolution_clock::now() - this->timer_duration > timer_start);
}

/** \brief Reset the start time of the timer. */
void arwain::Timers::CountdownTimer::reset()
{
    this->timer_start = std::chrono::high_resolution_clock::now();
}
