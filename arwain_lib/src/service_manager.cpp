#include <iostream>
#include <memory>

#include "arwain/service_manager.hpp"
#include "arwain/service_interface.hpp"

/** \brief Register a service so it can be retrieved later.
 * The caller must remember the name they used so they can unregister later.
 * \param service_ref: Point to ArwainJob base interface (usually just "this" is sufficient)
 * \param service_name: String description that can be known to other services and used to get the pointer.
 */
void ServiceManager::register_service(IArwainService* service_ref, const std::string& service_name)
{
    ServiceManager::services[service_name] = service_ref;
}

/** \brief Unregister a service by name. It is removed from the internal map
 * and subsequent requests for it will return a null pointer.
 * \param service_key: The name of the requested service.
 */
void ServiceManager::unregister_service(const std::string& service_key)
{
    ServiceManager::services.erase(service_key);
}
