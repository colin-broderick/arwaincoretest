#include <iostream>

#include "arwain_thread.hpp"

/** \brief Sets the name of the underlying thread.
 * \param name The name to assign to the thread, max 12 characters.
 */
void ArwainThread::set_name(const std::string& name)
{
    int rc = pthread_setname_np(this->native_handle(), name.data());
    if (rc != 0)
    {
        throw std::runtime_error{"Failed to set thread name."};
    }
}

/** \brief Retrieve the thread name from the underlying native thread handle.
 * \return String representation of the thread name.
 */
std::string ArwainThread::get_name()
{
    char name[16] = {0};
    int rc = pthread_getname_np(this->native_handle(), name, 16);
    if (rc != 0)
    {
        throw std::runtime_error{"Failed to get thread name."};
    }
    return std::string{name};
}
