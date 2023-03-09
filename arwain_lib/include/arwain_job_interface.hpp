#ifndef _ARWAIN_JOB_INTERFACE
#define _ARWAIN_JOB_INTERFACE

#include "event_manager.hpp"

class ArwainJobInterface
{
  public:
        virtual bool init() = 0;
        virtual bool join() = 0;
  protected:
        virtual void core_setup() = 0;
        virtual void run() = 0;
        virtual void run_idle() = 0;
        virtual void run_inference() = 0;
};

class ArwainJob : protected ArwainJobInterface
{
    private:
        uint64_t deregistration_key;
    protected:
        arwain::OperatingMode mode = arwain::OperatingMode::Idle;
        ArwainJob()
        {
            deregistration_key = EventManager::switch_mode_event.add_callback(
                std::bind(&ArwainJob::set_mode, this, std::placeholders::_1)
            );
        }
        void set_mode(arwain::OperatingMode new_mode)
        {
            mode = new_mode;
        }
        ~ArwainJob()
        {
            EventManager::switch_mode_event.remove_callback(deregistration_key);
        }
};

#endif
