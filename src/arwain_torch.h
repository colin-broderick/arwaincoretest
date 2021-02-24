#ifndef ARWAIN_TORCH_H
#define ARWAIN_TORCH_H

#include <string>
#include <vector>

class Torch
{
    private:
        std::string filename;
        std::vector<int> input_size;

    public:
        Torch(std::string model_file_name);
        Torch(std::string model_file_name, std::vector<int> shape);
        std::vector<double> infer(float data[1][6][200]);
};

#endif
