#ifndef _ARWAIN_THREAD_HPP
#define _ARWAIN_THREAD_HPP

#include <thread>
#include <vector>

/** \brief A wrapper around std::thread which provides the ability to set thread name. */
class ArwainThread : public std::thread
{
    public:
        /** \brief Does exactly what std::thread does.
         * \param func A callable object (generally a function), which will be run in a new thread.
         */
        template <class Callable>
        ArwainThread(Callable func)
        : std::thread(func)
        {

        };

        /** \brief Creates a thread, and assigns a name.
         * \param func A callable object (generally a function), which will be run in a new thread.
         * \param name_ A name for the thread, max 16 characters.
         */
        template <class Callable>
        ArwainThread(Callable func, const std::string& name_)
        : std::thread(func)
        {
            this->set_name(name_);
        };

        /** \brief Creates a thread, and assignes a name.
         * \param func A callable object (generally a function), which will be run in a new thread.
         * \param name_ A name for the thread, max 16 characters.
         * \param args A collection of arguments to pass to the threaded .
         */
        template <class Callable, class... Args>
        ArwainThread(Callable func, const std::string& name_, Args&&... args)
        : std::thread(func, args...)
        {
            this->set_name(name_);
        };

        std::string get_name();

    private:
        void set_name(const std::string& name);
};

#endif
