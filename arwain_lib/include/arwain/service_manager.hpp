#ifndef _ARWAIN_SERVICES_HPP
#define _ARWAIN_SERVICES_HPP

#include <string>

#include "arwain/job_interface.hpp"

namespace ServiceManager
{
    void register_service(IArwainJob* service_ref, const std::string& service_name);
    void unregister_service(const std::string& service_name);
    IArwainJob* get_service(const std::string& service_name);
}

#endif
