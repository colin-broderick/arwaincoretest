#ifndef _ARWAIN_BUFFER_HPP
#define _ARWAIN_BUFFER_HPP

#include <deque>
#include <mutex>

class Vector3;
struct ImuData;

/** \brief Memory safe wrapper of std::deque which automatically pops the
 * first element to maintain a fixed size.
 * 
 * This container is primarily designed to meet the needs of the ArwainJob
 * threads. They store data for a fixed period of time (i.e. these containers
 * have fixed length), and the oldest element in the container is automatically
 * removed each time new data is added.
 * 
 * \tparam DataType; the type to be stored in the container.
 * \tparam BufferSize; the container maintains this fixed size.
 */
template <class DataType, uint32_t BufferSize>
class GlobalBuffer
{
    public:
        /** \brief Initializes the container with a fixed size, containing
         * zero-initialized DataType objects.
         */
        GlobalBuffer()
        {
            data = std::deque<DataType>(BufferSize);
        }

        /** \brief Get a copy of internal buffer as an std::deque.
         * \return std::deque<DataType>, a copy of the internal buffer.
         */
        std::deque<DataType> get_data()
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            return data;
        }

        /** \brief Add an element to the end of the container. The first element
         * is automatically popped from the front of the container.
         */
        void push_back(const DataType& new_data)
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            data.push_back(new_data);
            data.pop_front();
        }

        /** \brief Get a copy of the last element in the container. 
         * \return A copy of the last element in the container.
         */
        DataType back()
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            return data.back();
        }

        /** \brief Remove all elements from the container.
         * The container maintains its size, so is now filled with zero-initialized
         * objects of type DataType.
         */
        void clear()
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            data = std::deque<DataType>(BufferSize);
        }

    private:
        /** \brief All accessors lock this mutex to provide memory safety. */
        std::mutex lock;
        
        /** \brief Internal storage, std::deque of fixed size. */
        std::deque<DataType> data;
};

#endif
