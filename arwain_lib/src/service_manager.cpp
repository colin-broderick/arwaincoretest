#include <iostream>
#include <memory>

#include "arwain/service_manager.hpp"
#include "arwain/job_interface.hpp"

namespace // Anonymous
{
    std::unordered_map<std::string, IArwainJob*> services;
    uint64_t current_key = 0;
}

namespace ServiceManager
{
    /** \brief Register a service so it can be retrieved later.
     * The caller must remember the name they used so they can unregister later.
     * \param service_ref: Point to ArwainJob base interface (usually just "this" is sufficient)
     * \param service_name: String description that can be known to other services and used to get the pointer.
     */
    void register_service(IArwainJob* service_ref, const std::string& service_name)
    {
        services[service_name] = service_ref;
    }

    /** \brief Unregister a service by name. It is removed from the internal map
     * and subsequent requests for it will return a null pointer.
     * \param service_key: The name of the requested service.
     */
    void unregister_service(const std::string& service_key)
    {
        services.erase(service_key);
    }

    /** \brief Get a handle to a service using its name.
     * \param service_key: The name of the requested service.
     * \return IArwainJob pointer to service if found, otherwise nullptr,
     */
    IArwainJob* get_service(const std::string& service_key)
    {
        if (services.contains(service_key))
        {
            return services.at(service_key);
        }
        else
        {
            return nullptr;
        }
    }
}
