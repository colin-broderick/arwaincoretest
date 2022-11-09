/*
The functionality to set affinity is not yet fully implemented, and does not appear to work in all environments.
The feature will be added later if it is needed. This early draft is stored so we can refer to it at that time.
*/

/** \brief Assign core affinity to the thread.
 * This can fail in the following ways:
 * 1. A core number is passed which corresponds to a non-existant core, e.g. a 4 is passed when there are only two cores.
 * 2. An unknown error is produced by the operating system when setting core affinity.
 * \param cores A vector of ints, where each int represents a CPU core, with cores indexed starting at zero.
 * \throw std::exception In the case that invalid core numbers are supplied.
 */
void ArwainThread::set_processor_affinity(const std::vector<int>& cores)
{
    // If AllCores are selected, there is nothing to do.
    if (cores.size() == 1 && cores[0] == ArwainThread::AllCores)
    {
        return;
    }

    // Work out how many cores there are.
    int cpu_count = std::thread::hardware_concurrency();

    // Make sure all ints are less than core number
    for (const int& cpu_number : cores)
    {
        if (cpu_number >= cpu_count - 1 || cpu_count < 0)
        {
            throw std::invalid_argument{"You tried to set an invalid core number"};
        }
    }

    // Create a CPU set and use pthreadsetaffinity to apply it to this->.
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (const int& cpu_number : cores)
    {
        CPU_SET(cpu_number, &cpuset);
    }

    int rc = pthread_setaffinity_np(this->native_handle(), sizeof(cpu_set_t), &cpuset);
    if (rc != 0)
    {
        throw std::invalid_argument{"Unkown error settings CPU affinity"};
    }
}
