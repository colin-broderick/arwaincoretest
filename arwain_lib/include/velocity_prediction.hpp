#ifndef PREDICT_VELOCITY_H
#define PREDICT_VELOCITY_H

#include "arwain_job_interface.hpp"
#include "arwain_thread.hpp"
#include "logger.hpp"

#if USE_NCS2
#include "ncs2_inferrer.hpp"
#else // Using tflite inference
#include "tf_inferrer.hpp"
    #include "tensorflow/lite/interpreter.h"
    #include "tensorflow/lite/kernels/register.h"
    #include "tensorflow/lite/model.h"
    #include "tensorflow/lite/tools/gen_op_registration.h"
#endif

class PositionVelocityInference : protected ArwainJob
{
    TESTABLE:
        void run_inference();
        void run_idle();
        void core_setup();
        void setup_inference();
        void run();
        void cleanup_inference();
        #if USE_NCS2
        NCS2Inferrer inferrer;
        #endif

        bool ready_for_inference = false;
        #if USE_NCS2
        #else // USE_TF
        std::unique_ptr<tflite::FlatBufferModel> model;
        tflite::ops::builtin::BuiltinOpResolver resolver;
        std::unique_ptr<tflite::Interpreter> interpreter;
        float* input;
        #endif

        // Local copy of IMU buffer data.
        std::deque<ImuData> imu;

        Vector3 position;
        Vector3 velocity;

        arwain::Logger velocity_file;
        arwain::Logger position_file;

        ArwainThread job_thread;
        #if USE_NCS2
        ArwainThread ncs2_thread;
        #endif

        uint64_t last_inference_time = 0;

    public:
        PositionVelocityInference();
        bool ready();
        bool init();
        bool join();
};

#endif
