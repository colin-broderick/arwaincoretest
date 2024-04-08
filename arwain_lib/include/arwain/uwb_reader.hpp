#ifndef _ARWAIN_UWB_READER_HPP
#define _ARWAIN_UWB_READER_HPP

#include <arwain/logger.hpp>

#include <uubla/uubla.hpp>

#include "arwain/job_interface.hpp"
#include "arwain/service_interface.hpp"

class UublaWrapper final : public ArwainJob, protected IArwainJobSpec, public IArwainService
{
    private:
        void run_inference() override;
        void run() override;
        void setup_inference() override;
        void run_idle() override;
        void core_setup() override;
        bool cleanup_inference() override;

    private:
        std::jthread job_thread;
        arwain::Logger uwb_log;
        std::unique_ptr<UUBLA::Network> m_uubla;
        uint64_t start_inertial_event_key = 0;
        uint64_t stop_inertial_event_key = 0;

    public:
        UublaWrapper();
        ~UublaWrapper();

        bool network_contains(int node_id);
        void fix_node_at(int node_id, const Vector3& new_position);
        static inline std::string service_name = "UublaWrapper";
        bool join() override;
        double get_distance(const int position);
        Vector3 get_own_position() const;
        Vector3 get_node_position(const std::string& node_name) const;
        std::jthread serial_reader_th;
};

#endif
