#ifndef _ARWAIN_EVENT_MANAGER
#define _ARWAIN_EVENT_MANAGER

#include <map>
#include <functional>
#include <mutex>

#include "arwain_modes.hpp"

template <class FuncSig, class InvokeArg>
class Event
{
    private:
        std::mutex lock;
        uint64_t current_key = 0;
        std::map<uint64_t, std::function<FuncSig>> callbacks;

    public:

        /** \brief Register a new callback. */
        uint64_t add_callback(std::function<FuncSig> func)
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            current_key++;
            callbacks[current_key] = func;
            return current_key;
        }
        
        std::size_t size()
        {
            return callbacks.size();
        }

        /** \brief Call all registered callbacks with the provided argument. */
        void invoke(InvokeArg arg)
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            for (auto [key, func] : callbacks)
            {
                func(arg);
            }
        }

        /** \brief Removes a callback from the action if the provided key is known. 
         * \param key A deregistration key which will have been provided when the callback was registered.
         */
        void remove_callback(uint64_t key)
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            callbacks.erase(key);
        }

        /** \brief Erase all callbacks from the action. */
        void clear()
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            callbacks.clear();
        }
};

namespace EventManager
{
    inline Event<void(arwain::OperatingMode), arwain::OperatingMode> switch_mode_event;
}

class StandAloneModeRegistrar
{
    TESTABLE:
        uint64_t deregistration_key = 0;
        arwain::OperatingMode mode = arwain::OperatingMode::Idle;
        void callback(arwain::OperatingMode new_mode)
        {
            mode = new_mode;
        }

    public:
        StandAloneModeRegistrar()
        {
            deregistration_key = EventManager::switch_mode_event.add_callback(
                std::bind(&StandAloneModeRegistrar::callback, this, std::placeholders::_1)
            );
        }
        ~StandAloneModeRegistrar()
        {
            EventManager::switch_mode_event.remove_callback(deregistration_key);
        }
        arwain::OperatingMode get_mode() const
        {
            return mode;
        }
};

#endif
