#ifndef _ARWAIN_COMMAND_LINE_HPP
#define _ARWAIN_COMMAND_LINE_HPP

#include "arwain/job_interface.hpp"

class PositionVelocityInference;

class ArwainCLI : public ArwainJob, protected IArwainJobSpec
{
    private:
        void run() override;
        static constexpr int s2i(std::string_view input);
        bool switch_to_exit_mode();
        std::string report_current_mode() const;
        bool switch_to_idle_autocal_mode();
        void switch_to_inference_mode();
        void core_setup() override;
        void force_switch_to_idle_autocal_mode();
        bool switch_to_gyro_calib_mode();
        void fail_to_switch_to(arwain::OperatingMode mode);
        bool switch_to_mag_calib_mode();
        bool switch_to_accel_calib_mode();
        bool switch_to_data_collection_mode();
        bool set_folder_name(const std::string& input);
        void parse_cli_input(const std::string& input);
        std::shared_ptr<std::istream> in_stream;
        void run_inference() override;
        void run_idle() override;
        void setup_inference() override;
        bool cleanup_inference() override;
        
    private:
        std::jthread job_thread;

    public:
        ArwainCLI();
        ArwainCLI(const std::istream& stream);
        bool init();
        bool join();
};

#endif
