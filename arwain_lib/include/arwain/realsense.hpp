#ifndef _ARWAIN_REALSENSE_HPP
#define _ARWAIN_REALSENSE_HPP

#include <thread>

#include "arwain/tests.hpp"
#include "arwain/logger.hpp"
#include "arwain/job_interface.hpp"

#include "t265.hpp"

class CameraController : public ArwainJob, protected IArwainJobSpec
{
    private:
        void setup_data_collection();
        void cleanup_data_collection();
        void core_setup() override;
        void run_data_collection();
        void run_idle() override;
        void run() override;

    private:
        std::jthread job_thread;
        T265* camera;
        arwain::Logger camera_position_log;

    public:
        CameraController();
        void join() override;
};

#endif