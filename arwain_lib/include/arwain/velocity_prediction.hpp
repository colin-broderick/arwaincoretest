#ifndef PREDICT_VELOCITY_H
#define PREDICT_VELOCITY_H

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <tensorflow/lite/tools/gen_op_registration.h>

#include "arwain/arwain.hpp"
#include "arwain/service_interface.hpp"

class PositionVelocityInference : public ArwainJob, protected IArwainJobSpec, public IArwainService
{
    private:
        void run_inference() override;
        void run_idle() override;
        void core_setup() override;
        void setup_inference() override;
        void run() override;
        bool cleanup_inference() override;

        std::unique_ptr<tflite::FlatBufferModel> model;
        tflite::ops::builtin::BuiltinOpResolver resolver;
        std::unique_ptr<tflite::Interpreter> interpreter;
        float* input;

        // Local copy of IMU buffer data.
        std::deque<ImuData> imu;

        Vector3 position;
        Vector3 velocity;

        arwain::Logger velocity_file;
        arwain::Logger position_file;

        std::jthread job_thread;

        uint64_t last_inference_time = 0;

    public:
        PositionVelocityInference();
        ~PositionVelocityInference();
        bool join() override;
        static inline std::string service_name = "PositionInference";
        Vector3 get_position() const;
};

#endif
