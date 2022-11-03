#ifndef _ARWAIN_THREAD_HPP
#define _ARWAIN_THREAD_HPP

#include <thread>
#include <vector>

/** \brief A wrapper around std::thread which provides the ability to set thread name and core affinity. */
class ArwainThread : public std::thread
{
    public:
        const static int AllCores = -1;

        /** \brief Does exactly what std::thread does.
         * \param func A callable object (generally a function), which will be run in a new thread.
         */
        template <class Callable>
        ArwainThread(Callable func)
        : std::thread(func)
        {

        };

        /** \brief Creates a thread, and assigned a name and core affinity.
         * \param func A callable object (generally a function), which will be run in a new thread.
         * \param name_ A name for the thread, max 16 characters.
         * \param cores A vector of ints, where the ints index the cores to be assigned to the task, starting at zero.
         */
        template <class Callable>
        ArwainThread(Callable func, const std::string& name_, const std::vector<int>&& cores)
        : std::thread(func)
        {
            this->set_name(name_);
            this->set_processor_affinity(cores);
        };

        /** \brief Creates a thread, and assigned a name and core affinity.
         * \param func A callable object (generally a function), which will be run in a new thread.
         * \param name_ A name for the thread, max 16 characters.
         * \param cores A vector of ints, where the ints index the cores to be assigned to the task, starting at zero.
         * \param args A collection of arguments to pass to the threaded .
         */
        template <class Callable, class... Args>
        ArwainThread(Callable func, const std::string& name_, const std::vector<int>&& cores, Args&&... args)
        : std::thread(func, args...)
        {
            this->set_name(name_);
            this->set_processor_affinity(cores);
        };

        std::string get_name();

    private:
        void set_processor_affinity(const std::vector<int>& cores);
        void set_name(const std::string& name);
};

#endif
