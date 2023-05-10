#ifndef _ARWAIN_UWB_READER_HPP
#define _ARWAIN_UWB_READER_HPP

#include "arwain/thread.hpp"
#inlcude "arwain/job_interface"

#include "uubla/uubla.hpp"

class UublaWrapper : public ArwainJob
{
    TESTABLE:
        void run_inference();
        void run();
        void setup_inference();
        void run_idle();
        void core_setup();
        void cleanup_inference();
        bool init();

    private:
        ArwainThread job_thread;
        ArwainThread solver_th;
        UUBLA::Network* uubla;

    public:
        UublaWrapper();
        void join();
        double get_distance(const int position);
};

#endif
