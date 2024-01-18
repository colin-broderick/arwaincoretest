#ifndef _HYBRID_POS_HPP
#define _HYBRID_POS_HPP

#include <arwain/timers.hpp>

#include "arwain/job_interface.hpp"

class HybridPositioner : public ArwainJob, protected IArwainJobSpec
{
    public:
        HybridPositioner()
        {
            init();
        }

        bool init() override
        {
            core_setup();
            job_thread = ArwainThread{&HybridPositioner::run, "arwain_hybr_th", this};
            return true;
        }

        bool join() override
        {
            if (job_thread.joinable())
            {
                job_thread.join();
                return true;
            }
            return false; // This won't need to return a bool if it's based on jthread and autojoins - it either returns or fails, not both.
        }

        void run() override
        {
            // TODO Convert all the run functions to take a stop_token - will require fixing arwainthread.
            // THis is a stub to satisfy the IArwainJobSpec interface - will be corrected when we
            // update all inheriting classes to use jthread and std::stop_token
            while (mode != arwain::OperatingMode::Terminate)
            {
                switch (mode)
                {
                    case arwain::OperatingMode::GyroscopeCalibration:
                    case arwain::OperatingMode::MagnetometerCalibration:
                    case arwain::OperatingMode::TestStanceDetector:
                    case arwain::OperatingMode::AccelerometerCalibration:
                    case arwain::OperatingMode::SelfTest:
                    case arwain::OperatingMode::Idle:
                        run_idle();
                        break;
                    case arwain::OperatingMode::Inference:
                        run_inference();
                        break;
                    default:
                        sleep_ms(10);
                        break;
                }
            }
        }

        void core_setup() override
        {
            // Currently no core_setup to do; might come later.
        }

        void run_inference() override
        {
            setup_inference();

            Timers::IntervalTimer<std::chrono::milliseconds> loop_scheduler{100, "hybrid_position_timer"}; // TODO Consider timing.

            while (mode == arwain::OperatingMode::Inference)
            {
                // TODO
                // Get position from inferrer
                // Get position from UWB
                // Fuse positions ....
                // Probably need to also consider orientation.
                sleep_ms(10);
            }

            cleanup_inference();
        }

        void run_idle() override
        {
            while (mode == arwain::OperatingMode::Idle)
            // TODO This loop condition is flawed since several modes can bring us here,
            // and several modes can bring us out.
            {
                sleep_ms(10);
            }
        }

        void setup_inference() override
        {
            hybrid_pos_log.open(arwain::folder_date_string + "/hyrid_position.txt");
            hybrid_pos_log << "time x y z\n"; // TODO Consider exactly what to log.
        }

        void cleanup_inference() override
        {
            hybrid_pos_log.close();
        }

    private:
        ArwainThread job_thread;
        arwain::Logger hybrid_pos_log;
};

#endif
