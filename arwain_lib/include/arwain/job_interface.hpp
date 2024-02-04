#ifndef _ARWAIN_JOB_INTERFACE
#define _ARWAIN_JOB_INTERFACE

#include <arwain/event_manager.hpp>

#include "arwain/events.hpp"

class IArwainJob
{
    public:
        virtual bool join() = 0;
        virtual void run() = 0;
};

class IArwainJobSpec
{
    public:
        virtual void core_setup() = 0;
        virtual void run_inference() = 0;
        virtual void run_idle() = 0;
        virtual void setup_inference() = 0;
        virtual bool cleanup_inference() = 0;
};

class ArwainJob : public IArwainJob
{
    public:
        arwain::OperatingMode get_mode() const
        {
            return mode;
        }

        uint64_t deregistration_key = 0;
        
        arwain::OperatingMode mode = arwain::OperatingMode::Idle;
        ArwainJob()
        {
            deregistration_key = arwain::Events::switch_mode_event.add_callback(
                std::bind(&ArwainJob::set_mode, this, std::placeholders::_1)
            );
        }
        void set_mode(arwain::OperatingMode new_mode)
        {
            mode = new_mode;
        }
        ~ArwainJob()
        {
            arwain::Events::switch_mode_event.remove_callback(deregistration_key);
        }
};

#endif
