#ifndef _ARWAIN_COMMAND_LINE_HPP
#define _ARWAIN_COMMAND_LINE_HPP
#include "arwain_thread.hpp"
#include "arwain.hpp"

class PositionVelocityInference;

class ArwainCLI
{
    TESTABLE:
        void run();
        static constexpr int s2i(std::string_view input);
        bool switch_to_exit_mode();
        void report_current_mode();
        bool switch_to_idle_autocal_mode();
        void switch_to_inference_mode();
        void core_setup();
        void force_switch_to_idle_autocal_mode();
        bool switch_to_gyro_calib_mode();
        void fail_to_switch_to(arwain::OperatingMode mode);
        bool switch_to_mag_calib_mode();
        bool switch_to_accel_calib_mode();
        bool switch_to_data_collection_mode();
        bool set_folder_name(const std::string& input);
        void parse_cli_input(const std::string& input);

    private:
        PositionVelocityInference* velocity_inference_handle = nullptr;
        ArwainThread job_thread;

    public:
        bool set_velocity_inference_pointer(PositionVelocityInference& velocity);
        ArwainCLI();
        bool init();
        void join();
};

#endif
