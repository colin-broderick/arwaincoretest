#ifndef PREDICT_VELOCITY_H
#define PREDICT_VELOCITY_H

#include "arwain.hpp"

class PositionVelocityInference
{
    TESTABLE:
        void run_inference();
        void run_idle();
        void core_setup();
        void setup_inference();
        void run();
        void cleanup_inference();
        #if USE_NCS2
        void py_inference();
        #endif

    private:
        #if USE_NCS2
        // Socket for comm with Python script to manage NCS2.
        const std::string inference_tcp_socket = "tcp://*:5555";
        void* context = nullptr;
        void* responder = nullptr;
        // Socket request and response buffers
        std::stringstream request;
        char response_buffer[50];
        #else // USE_TF
        std::unique_ptr<tflite::FlatBufferModel> model;
        tflite::ops::builtin::BuiltinOpResolver resolver;
        std::unique_ptr<tflite::Interpreter> interpreter;
        float* input;
        #endif

        // Local copy of IMU buffer data.
        std::deque<Vector6> imu;

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
        void join();
        bool init();
};

#endif
