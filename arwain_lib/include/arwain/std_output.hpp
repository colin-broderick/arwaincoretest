#ifndef _ARWAIN_STD_OUTPUT_HPP
#define _ARWAIN_STD_OUTPUT_HPP

#include <tuple>
#include <thread>

#include "arwain/job_interface.hpp"

class StanceDetection;

/** \brief Periodically logs status messages to stdout.
 * Useful for debugging or testing, but probably not wanted at runtime.
 * Output is of a form that can be easily piped to other processes.
 */
class DebugPrints final : public ArwainJob, protected IArwainJobSpec
{
    private:
        void run() override;
        void run_inference() override;
        void run_idle() override;
        void setup_inference() override;
        bool cleanup_inference() override;
        void core_setup() override;
        
    private:
        StanceDetection* stance_detection_handle = nullptr;
        std::jthread job_thread;

    public:
        bool set_stance_detection_pointer(StanceDetection& stance);
        DebugPrints();
        bool join() override;
};

#endif
