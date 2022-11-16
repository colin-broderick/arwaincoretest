#ifndef _ARWAIN_STD_OUTPUT_HPP
#define _ARWAIN_STD_OUTPUT_HPP

#include <tuple>

/** \brief Periodically logs status messages to stdout.
 * Useful for debugging or testing, but probably not wanted at runtime.
 * Output is of a form that can be easily piped to other processes.
 */
class DebugPrints
{
    TESTABLE:
        void run();
        void run_inference();
        void run_idle();
        void setup_inference();
        void cleanup_inference();
        void core_setup();
        
    private:
        ArwainThread job_thread;

    public:
        DebugPrints();
        bool init();
        void join();
};

#endif
