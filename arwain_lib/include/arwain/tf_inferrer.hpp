#ifndef _GREEVE_TF_INFERRER
#define _GREEVE_TF_INFERRER

#include <filesystem>

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <tensorflow/lite/tools/gen_op_registration.h>

#include "arwain/vel_infer_interface.hpp"

class TFInferrer final : public I_VelInferrer
{
private:
    std::unique_ptr<tflite::FlatBufferModel> model;
    tflite::ops::builtin::BuiltinOpResolver resolver;
    std::unique_ptr<tflite::Interpreter> interpreter;
    std::unique_ptr<float> input;

public:
    TFInferrer(const std::filesystem::path model_file)
    {
        model = tflite::FlatBufferModel::BuildFromFile(model_file.c_str());
        if (!model)
        {
            std::cerr << "Failed to map model - does the file exist?\n";
            throw "TFLite model mapping failed";
        }

        tflite::InterpreterBuilder(*model.get(), resolver)(&interpreter);
        interpreter->AllocateTensors();
        input = std::unique_ptr<float>(interpreter->typed_input_tensor<float>(0));
    }

    Vector3 infer(std::deque<ImuData> &imu_data)
    {
        unsigned int input_index = 0;
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 200; j++)
            {
                *(input.get() + input_index) = imu_data[j][i];
                input_index++;
            }
        }
        interpreter->Invoke();
        float* output = interpreter->typed_output_tensor<float>(0);
        return {output[0], output[1], output[2]};
    }
};

#endif