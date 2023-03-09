#ifndef _ARWAIN_STD_OUTPUT_HPP
#define _ARWAIN_STD_OUTPUT_HPP

#include <tuple>

#include "arwain_thread.hpp"
#include "arwain_job_interface.hpp"

class StanceDetection;

/** \brief Periodically logs status messages to stdout.
 * Useful for debugging or testing, but probably not wanted at runtime.
 * Output is of a form that can be easily piped to other processes.
 */
class DebugPrints : protected ArwainJob
{
    TESTABLE:
        void run();
        void run_inference();
        void run_idle();
        void setup_inference();
        void cleanup_inference();
        void core_setup();
        
    private:
        StanceDetection* stance_detection_handle = nullptr;
        ArwainThread job_thread;

    public:
        bool set_stance_detection_pointer(StanceDetection& stance);
        DebugPrints();
        bool init();
        bool join();
};

#endif
