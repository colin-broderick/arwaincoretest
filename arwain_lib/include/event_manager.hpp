#ifndef _ARWAIN_EVENT_MANAGER
#define _ARWAIN_EVENT_MANAGER

#include "arwain.hpp"

template <class FuncSig, class InvokeArg>
class Event
{
    private:
        uint64_t current_key = 0;
        std::map<uint64_t, std::function<FuncSig>> callbacks;

    public:

        /** \brief Register a new callback. */
        uint64_t add_callback(std::function<FuncSig> func)
        {
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
            callbacks.erase(key);
        }

        /** \brief Erase all callbacks from the action. */
        void clear()
        {
            callbacks.clear();
        }
};

namespace EventManager
{
    inline Event<void(arwain::OperatingMode), arwain::OperatingMode> switch_mode_event;
}

#endif
