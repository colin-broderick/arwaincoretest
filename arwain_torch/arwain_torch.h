#ifndef ARWAIN_TORCH_H
#define ARWAIN_TORCH_H

#include <string>
#include <vector>

namespace arwain
{
    class Torch
    {
        private:
            std::string filename;
            std::vector<int> input_size;
            float data[1][6][200];
            void torch_array_from_deque(float data[1][6][200], std::deque<std::array<double, 6>> imu);

        public:
            Torch(std::string model_file_name);
            Torch(std::string model_file_name, std::vector<int> shape);
            std::vector<double> infer(float data[1][6][200]);
            std::vector<double> infer(const std::deque<std::array<double, 6>> &imu);
    };
}

#endif
