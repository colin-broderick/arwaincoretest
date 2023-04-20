#ifndef _ARWAIN_BUFFER_HPP
#define _ARWAIN_BUFFER_HPP

#include <deque>
#include <mutex>

class Vector3;
struct ImuData;

template <class DataType, int BufferSize>
class GlobalBuffer
{
    public:
        GlobalBuffer()
        {
            data = std::deque<DataType>(BufferSize);
        }

        std::deque<DataType> get_data()
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            return data;
        }

        void push_back(const DataType& new_data)
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            data.push_back(new_data);
            data.pop_front();
        }

        DataType back()
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            return data.back();
        }

        void clear()
        {
            std::lock_guard<std::mutex> lock_guard{lock};
            data = std::deque<DataType>(BufferSize);
        }

    private:
        std::mutex lock;
        std::deque<DataType> data;
};

#endif
