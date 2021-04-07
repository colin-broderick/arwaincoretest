#ifndef ARWAIN_LOGGER_H
#define ARWAIN_LOGGER_H

#include <string>
#include <fstream>
#include <mutex>

namespace arwain
{
    /** \brief This is just a wrapper for std::ofstream which adds thread safety via mutex lock guard. */
    class Logger
    {
        public:
            Logger();
            Logger(const std::string& filename);
            bool open(const std::string& filename);
            void close();
            bool is_open() const;
            std::string get_filename() const;

        private:
            std::mutex guard;
            std::string filename;
            std::ofstream output_file;
            template<typename T>
            friend arwain::Logger& operator<<(arwain::Logger& stream, const T& thing)
            {
                std::lock_guard<std::mutex> lock{stream.guard};
                stream.output_file << thing;
                return stream;
            }
    };
}

#endif
