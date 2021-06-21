/*
This is the start of an attempt to make a thread safe wrapper for the std::deque container.
*/

// TODO Need index operators

#ifndef SAFE_BUFFER_H
#define SAFE_BUFFER_H

#include <mutex>
#include <deque>

namespace arwain
{
    template <typename T> class Buffer
    {
        public:
            Buffer() { /* Empty constructor */ }

            Buffer(unsigned int size)
            {
                internal_buffer = std::deque<T>{size};
            }

            Buffer(T new_data)
            {
                internal_buffer = new_data;
            }

            void push_back(T element)
            {
                std::lock_guard<std::mutex> lock{mlock};
                internal_buffer.push_back(element);
            }

            void pop_front()
            {
                std::lock_guard<std::mutex> lock{mlock};
                internal_buffer.pop_front();
            }

            T back()
            {
                std::lock_guard<std::mutex> lock{mlock};
                return internal_buffer.back();
            }

            unsigned int size()
            {
                std::lock_guard<std::mutex> lock{mlock};
                return internal_buffer.size();
            }

            Buffer<T> operator=(const Buffer<T>& b)
            {
                std::lock_guard<std::mutex> lock{mlock};
                return Buffer{internal_buffer};
            }

        private:
            std::mutex mlock;
            std::deque<T> internal_buffer;
    };
}

#endif
