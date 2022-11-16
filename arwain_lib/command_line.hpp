#ifndef _ARWAIN_COMMAND_LINE_HPP
#define _ARWAIN_COMMAND_LINE_HPP

class ArwainCLI
{
    TESTABLE:
        void run();
        static constexpr int s2i(std::string_view input);
        void switch_to_exit_mode();
        void report_current_mode();
        void switch_to_idle_autocal_mode();
        void switch_to_inference_mode();
        void core_setup();
        void force_switch_to_idle_autocal_mode();
        void switch_to_gyro_calib_mode();
        void fail_to_switch_to(arwain::OperatingMode mode);
        void switch_to_mag_calib_mode();
        void switch_to_accel_calib_mode();
        void switch_to_data_collection_mode();
        void set_folder_name(const std::string& input);
        void set_position_zero();
        void parse_cli_input(const std::string& input);

    private:
        ArwainThread job_thread;

    public:
        ArwainCLI();
        bool init();
        void join();
};

#endif
