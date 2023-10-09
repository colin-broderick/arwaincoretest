#ifndef _ARWAIN_EVENTS
#define _ARWAIN_EVENTS

#include <arwain/event.hpp>

#include "arwain/modes.hpp"

#include "vector3.hpp"

namespace arwain::Events
{
    /** \brief Subscribe to this event to trigger a callback when Arwain system mode changes. */
    inline Event<arwain::OperatingMode> switch_mode_event;

    /** \brief Subscribe to this event to trigger a callback when new Arwain velocity data is available. */
    inline Event<Vector3> new_arwain_velocity_event;

    /** \brief Subscribe to this event to trigger a callback when new Arwain position data is available. */
    inline Event<Vector3> new_arwain_position_event;

    /** \brief Subscribe to this event to trigger a callback when new UWB position data is available */
    inline Event<Vector3> new_uwb_position_event;
}

/** \brief This class exists to provide a means to track system mode in the cases where no
 * ArwainJobs are running. It has no particular utility outside of the Arwain codebase.
 * The class provides a simple callback which, registered to the switch_mode_event, which
 * updates internal mode state when the event is triggered. Useful for testing.
 */
class StandAloneModeRegistrar
{
    public:
        /** \brief The constructor registers the objects's callback() function with
         * the switch_mode_event event in the EventManager.
         */
        StandAloneModeRegistrar()
        {
            deregistration_key = arwain::Events::switch_mode_event.add_callback(
                std::bind(&StandAloneModeRegistrar::callback, this, std::placeholders::_1)
            );
        }
        
        /** \brief Deregisteres the event callback up destruction of object. */
        ~StandAloneModeRegistrar()
        {
            arwain::Events::switch_mode_event.remove_callback(deregistration_key);
        }

        /** \brief Get the current Arwain system mode.
         * \return Current Arwain system mode.
         */
        arwain::OperatingMode get_mode() const
        {
            return mode;
        }

    private:
        /** \brief Key used to deregister the callback when this class's destructor is called. */
        uint64_t deregistration_key = 0;

        /** \brief Current Arwain system mode, updated when the callback is called. */
        arwain::OperatingMode mode = arwain::OperatingMode::Idle;

    private:
        /** \brief Called when the switch_mode_event is triggered. Sets the internal system mode. */
        void callback(arwain::OperatingMode new_mode)
        {
            mode = new_mode;
        }
};

#endif
