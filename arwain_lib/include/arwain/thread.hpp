#ifndef _ARWAIN_THREAD_HPP
#define _ARWAIN_THREAD_HPP

#include <thread>
#include <vector>

/** \brief A wrapper around std::jthread which provides the ability to set thread name. */
class ArwainThread : public std::jthread
{
    public:
        enum class Cores
        {
            Core0 = 0, Core1, Core2, Core3, AllCores
        };

        ArwainThread() = default;

        /** \brief Does exactly what std::jthread does.
         * \param func A callable object (generally a function), which will be run in a new thread.
         */
        template <class Callable>
        ArwainThread(Callable func)
        : std::jthread(func)
        {

        };

        /** \brief Creates a thread, and assigns a name.
         * \param func A callable object (generally a function), which will be run in a new thread.
         * \param name_ A name for the thread, max 16 characters.
         */
        template <class Callable>
        ArwainThread(Callable func, const std::string& name_)
        : std::jthread(func)
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
        : std::jthread(func, args...)
        {
            this->set_name(name_);
        };

        void set_processor_affinity(const std::vector<ArwainThread::Cores>& cores);
        std::string get_name();

    private:
        void set_name(const std::string& name);
};

#endif
