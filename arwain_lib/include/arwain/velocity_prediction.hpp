#ifndef PREDICT_VELOCITY_H
#define PREDICT_VELOCITY_H

#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <tensorflow/lite/tools/gen_op_registration.h>

#include "arwain/arwain.hpp"

class PositionVelocityInference : public ArwainJob
{
    TESTABLE:
        void run_inference();
        void run_idle();
        void core_setup();
        void setup_inference();
        void run();
        void cleanup_inference();

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

        ArwainThread job_thread;

        uint64_t last_inference_time = 0;

    public:
        PositionVelocityInference();
        bool init();
        bool join();
};

#endif
