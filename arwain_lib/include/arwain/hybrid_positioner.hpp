#ifndef _HYBRID_POS_HPP
#define _HYBRID_POS_HPP

#include <arwain/timers.hpp>
#include <arwain/logger.hpp>

#include "arwain.hpp"
#include "arwain/job_interface.hpp"
#include "arwain/utils.hpp"

class HybridPositioner : public ArwainJob, protected IArwainJobSpec
{
    private:
        class PositionHybridizer
        {
            public:
                Vector3 position;
            public:
                Vector3 update(Vector3 inertial_position, Vector3 uwb_position);
                Vector3 get_position() const;
        };

    public:
        HybridPositioner();
        ~HybridPositioner();
        bool init() override;
        bool join() override;
        Vector3 get_position() const;

    private:
        PositionHybridizer hyb;
        std::jthread job_thread;
        arwain::Logger hybrid_pos_log;
        void new_inertial_velocity_callback(arwain::Events::Vector3EventWithDt uwb_position);
        void new_uwb_position_callback(arwain::Events::Vector3EventWithDt inertial_velocity);

    private:
        uint64_t vel_event_id = 0;
        uint64_t pos_event_id = 0;
    
    private:
        void run() override;
        void core_setup() override;
        void run_inference() override;
        void run_idle() override;
        void setup_inference() override;
        bool cleanup_inference() override;
};

#endif
