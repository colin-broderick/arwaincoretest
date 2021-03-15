#include <torch/script.h> // One-stop header.
#include <vector>
#include <string>
#include <array>

#include <iostream>
#include <memory>

#include "arwain_torch.h"

/** \brief Constructor
 * \param model_file_name The file name of the torchscript model to be loaded.
 */
arwain::Torch::Torch(std::string model_file_name)
{
    filename = model_file_name;
    input_size = {1, 6, 200};
}

/** \brief Constructor with option to specify model input shape.
 * \param model_file_name The file name of the torchscipt model to be loaded.
 * \param shape The shape of the model input tensor.
 */
arwain::Torch::Torch(std::string model_file_name, std::vector<int> shape)
{
    filename = model_file_name;
    input_size = {shape[0], shape[1], shape[2]};
}

/** \brief Do inference on a data deque, makes calling code cleaner.
 * \param data The data deque on which to do inference
 * \return A vector of doubles in the order x, y, z.
 */
std::vector<double> arwain::Torch::infer(const std::deque<std::array<double, 6>> &imu)
{
    this->torch_array_from_deque(data, imu);
    return this->infer(data);
}

/** \brief Do inference on a data array.
 * \param data The data array on which to do inference
 * \return A vector of doubles in the order x, y, z.
 */
std::vector<double> arwain::Torch::infer(float data[1][6][200])
{
    // Create model object. Only done once.
    static torch::jit::script::Module model = torch::jit::load(filename);
  
    // Create input data tensor.
    auto input_data = torch::from_blob(data, {1, 6, 200});
    std::vector<torch::jit::IValue> input = {input_data};
    
    // Create output tensor.
    at::Tensor output_tensor;
        
    // Do inference.
    output_tensor = model.forward(input).toTensor();
    double x = output_tensor[0][0].item().to<double>();
    double y = output_tensor[0][1].item().to<double>();
    double z = output_tensor[0][2].item().to<double>();

    return std::vector<double>{x, y, z};
}

/** \brief Converts the IMU deque into a C-style array for passing to the inference library.
 * \param[out] data C-style array of shape [1][6][200] in which to store result.
 * \param[in] imu The latest world IMU buffer.
 */
void arwain::Torch::torch_array_from_deque(float data[1][6][200], std::deque<std::array<double, 6>> imu)
{
    for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            for (int k = 0; k < 200; k++)
            {
                // Switch the order of gyro and acceleration, and put into array.
                data[i][j][k] = (float)(imu[k][(j+3)%6]);
            }
        }
    }
}