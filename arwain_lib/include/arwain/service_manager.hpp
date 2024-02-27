#ifndef _ARWAIN_SERVICES_HPP
#define _ARWAIN_SERVICES_HPP

#include <string>
#include <unordered_map>
#include "arwain/service_interface.hpp"

class ServiceManager
{
    private:
        static inline std::unordered_map<std::string, IArwainService*> services;
        static inline uint64_t current_key = 0;

    public:
        static void register_service(IArwainService* service_ref, const std::string& service_name);
        static void unregister_service(const std::string& service_name);

        /** \brief Get a handle to a service using its name.
         * \param service_key: The name of the requested service.
         * \return IArwainService pointer to service if found, otherwise nullptr,
         */
        template <class T>
        static T* get_service(const std::string& service_name)
        {
            static_assert(std::is_base_of<IArwainService, T>::value);

            if (ServiceManager::services.contains(service_name))
            {
                return static_cast<T*>(services.at(service_name));
            }
            else
            {
                return nullptr;
            }
        }
};

#endif
