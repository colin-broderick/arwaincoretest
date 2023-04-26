#ifndef _ARWAIN_EVENT_MANAGER
#define _ARWAIN_EVENT_MANAGER

#include <map>
#include <functional>
#include <mutex>

#include "arwain_modes.hpp"
#include "vector3.hpp"

template <class FuncSig, class InvokeArg> class Event;

template<class FuncSig, class InvokeArg>
class EventRegistrationHandle
{
    public:
        EventRegistrationHandle(uint64_t key, Event<FuncSig, InvokeArg>* key_provider_event)
        : m_key(key), m_event(key_provider_event)
        {
        }
        ~EventRegistrationHandle()
        {
            m_event->remove_callback(m_key);
        }
    private:
        uint64_t m_key = 0;
        Event<FuncSig, InvokeArg>* m_event;
};

/** \brief A synchronous event manager allowing an unlimited number of callback registrations.
 *
 * Callbacks are registered using add_callback(). Callbacks are deregistered using remove_callback().
 * Registered callbacks many be called using invoke(). Currently, only single-argument callbacks
 * are supported. Changes are planned to support asynchronous callbacks.
 * 
 * \tparam FuncSig: The signature of the expected callback functions.
 * \tparam InvokeArg: The type of the argument to be passed to callbacks.
 */
template <class FuncSig, class InvokeArg>
class Event
{
    public:
        /** \brief Register a new callback.
         * \param func A function pointer to the function to be called when the event
         * is triggered.
         * \return A callback registration key which is unique to this event. The key
         * should be  used to deregister the callback.
         */
        uint64_t add_callback(std::function<FuncSig> func)
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            current_key++;
            callbacks[current_key] = func;
            return current_key;
        }
        
        /** \brief Get the count of callbacks registered to this event. 
         * \return Count of registered callbacks.
         */
        std::size_t size()
        {
            return callbacks.size();
        }

        /** \brief Call all registered callbacks with the provided argument.
         * \param arg The value of the argument to pass to the callbacks.
         */
        void invoke(InvokeArg arg)
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            for (auto [key, func] : callbacks)
            {
                func(arg);
            }
        }

        /** \brief Removes a callback from the event if the provided key is known.
         * \param key A deregistration key which was provided when the callback was registered.
         */
        void remove_callback(uint64_t key)
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            callbacks.erase(key);
        }

        /** \brief Erase all callbacks from the event. */
        void clear()
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            callbacks.clear();
        }

    private:
        /** \brief All accessors lock this mutex to provide memory safety. */
        std::mutex lock;

        /** \brief Increments on each callback registration so that registartion 
         * keys are unique.
         */
        uint64_t current_key = 0;

        /** \brief A container for all registered callbacks. */
        std::map<uint64_t, std::function<FuncSig>> callbacks;
};

namespace EventManager
{
    /** \brief Subscribe to this event to trigger a callback when Arwain system mode changes. */
    inline Event<void(arwain::OperatingMode), arwain::OperatingMode> switch_mode_event;

    /** \brief Subscribe to this event to trigger a callback when new velocity data is available. */
    inline Event<void(Vector3), Vector3> new_arwain_velocity_event;

    /** \brief Subscribe to this event to trigger a callback when new position data is available. */
    inline Event<void(Vector3), Vector3> new_arwain_position_event;
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
            deregistration_key = EventManager::switch_mode_event.add_callback(
                std::bind(&StandAloneModeRegistrar::callback, this, std::placeholders::_1)
            );
        }
        
        /** \brief Deregisteres the event callback up destruction of object. */
        ~StandAloneModeRegistrar()
        {
            EventManager::switch_mode_event.remove_callback(deregistration_key);
        }

        /** \brief Get the current Arwain system mode.
         * \return Current Arwain system mode.
         */
        arwain::OperatingMode get_mode() const
        {
            return mode;
        }

    TESTABLE:
        /** \brief Key used to deregister the callback when this class's destructor is called. */
        uint64_t deregistration_key = 0;

        /** \brief Current Arwain system mode, updated when the callback is called. */
        arwain::OperatingMode mode = arwain::OperatingMode::Idle;

    TESTABLE:
        /** \brief Called when the switch_mode_event is triggered. Sets the internal system mode. */
        void callback(arwain::OperatingMode new_mode)
        {
            mode = new_mode;
        }
};

#endif
