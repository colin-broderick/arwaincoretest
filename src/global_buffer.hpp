#ifndef _ARWAIN_BUFFER_HPP
#define _ARWAIN_BUFFER_HPP

#include <deque>
#include <mutex>

class Vector3;
class Vector6;

template <class DataType, int BufferSize>
class GlobalBuffer
{
    public:
        GlobalBuffer()
        {
            // bool type_ok = (
            //     std::is_same<DataType, Vector3>::value
            //     && std::is_same<DataType, Vector6>::value
            // );

            // if (type_ok)
            // {
                data = std::deque<DataType>(BufferSize);
            // }
            // else
            // {
            //     throw std::invalid_argument{"GlobalBuffer template using with invalid template type"};
            // }
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
