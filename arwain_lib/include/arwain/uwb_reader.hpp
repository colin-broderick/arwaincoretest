#ifndef _ARWAIN_UWB_READER_HPP
#define _ARWAIN_UWB_READER_HPP

#include <uubla/uubla.hpp>

#include "arwain/thread.hpp"
#include "arwain/job_interface"

class UublaWrapper : public ArwainJob
{
    private:
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
