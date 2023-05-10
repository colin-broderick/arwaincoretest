#ifndef _ARWAIN_REALSENSE_HPP
#define _ARWAIN_REALSENSE_HPP

#include "arwain/tests.hpp"
#include "arwain/thread.hpp"
#include "arwain/logger.hpp"
#include "arwain/job_interface.hpp"

#include "t265.hpp"

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