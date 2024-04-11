#ifndef PREDICT_VELOCITY_H
#define PREDICT_VELOCITY_H

#include "arwain/arwain.hpp"
#include "arwain/service_interface.hpp"
#include "arwain/tf_inferrer.hpp"

class PositionVelocityInference final : public ArwainJob, protected IArwainJobSpec, public IArwainService
{
    private:
        void run_inference() override;
        void run_idle() override;
        void core_setup() override;
        void setup_inference() override;
        void run() override;
        bool cleanup_inference() override;

        std::unique_ptr<TFInferrer> inferrer;

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
