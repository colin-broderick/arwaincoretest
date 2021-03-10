#include <torch/script.h> // One-stop header.
#include <vector>
#include <string>
#include <array>

#include <iostream>
#include <memory>

#include "arwain_torch.h"

Torch::Torch(std::string model_file_name)
{
    filename = model_file_name;
    input_size = {1, 6, 200};
}

Torch::Torch(std::string model_file_name, std::vector<int> shape)
{
    filename = model_file_name;
    input_size = {shape[0], shape[1], shape[2]};
}
    
std::vector<double> Torch::infer(float data[1][6][200])
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

// int main(int argc, const char* argv[]) {
//     // Create model.
//     Torch model{"./xyzronin_v0-5_all2D_small.pt", {1, 6, 200, 1}};
    
//     // Create data.
//     float data[1][6][200][1];
//     for (int i = 0; i < 1; i++)
//     {
//       for (int j = 0; j < 6; j++)
//       {
//         for (int k = 0; k < 200; k++)
//         {
//           for (int l = 0; l < 1; l++)
//           {
//             data[i][j][k][l] = 1.0;
//           }
//         }
//       }
//     }

//     for (int i = 0; i < 20; i++)
//     {
//         // Get the result.
//         std::vector<double> result = model.infer(data);
//         // Print the result.
//         std::cout << result[0] << " " << result[1] << " " << result[2] << "\n";
//     }
// }