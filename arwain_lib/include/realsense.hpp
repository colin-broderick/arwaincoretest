#ifndef _ARWAIN_REALSENSE_HPP
#define _ARWAIN_REALSENSE_HPP

#include "t265.hpp"
#include "arwain_tests.hpp"
#include "arwain_thread.hpp"
#include "logger.hpp"
#include "arwain_job_interface.hpp"

class CameraController : public ArwainJob
{
    TESTABLE:
        void setup_data_collection();
        void cleanup_data_collection();
        void core_setup();
        void run_data_collection();
        void run_idle();
        void run();

    private:
        ArwainThread job_thread;
        T265* camera;
        arwain::Logger camera_position_log;

    public:
        CameraController();
        bool init();
        void join();

};

#endif