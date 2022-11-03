#ifndef _ARWAIN_THREAD_HPP
#define _ARWAIN_THREAD_HPP

#include <thread>
#include <vector>

class ArwainThread : public std::thread
{

    public:
        const static int AllCores = -1;

        template <class Callable>
        ArwainThread(Callable func)
        : std::thread(func)
        {

        };

        template <class Callable>
        ArwainThread(Callable func, const std::string& name_, const std::vector<int>&& cores)
        : std::thread(func)
        {
            this->set_name(name);
        };

        template <class Callable, class... Args>
        ArwainThread(Callable func, const std::string& name_, const std::vector<int>&& cores, Args&&... args)
        : std::thread(func, args...)
        {
            this->set_name(name);
        };

        std::string get_name();

    private:
        void set_processor_affinity(const std::vector<int>& cores);
        void set_name(const std::string& name);
        std::string name;
};

#endif
