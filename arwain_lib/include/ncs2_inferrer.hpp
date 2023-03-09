#ifndef _GREEVE_NCS2_INFERRER
#define _GREEVE_NCS2_INFERRER

#include "arwain_thread.hpp"
#include "vel_infer_interface.hpp"

#if !CHERI
#include <zmq.h>
#endif

#include <string>
#include <sstream>

#if CHERI
class NCS2Inferrer : public I_VelInferrer
{
    public:
        bool ready = true;
        Vector3 infer(const std::deque<ImuData>& imu_data)
        {
            (void)imu_data;
            return {0, 0, 0};
        }
        void init()
        {
            return;
        }
};
#else
class NCS2Inferrer : public I_VelInferrer
{
    TESTABLE:
        void* context = nullptr;
        void* responder = nullptr;
        const std::string inference_tcp_socket = "tcp://*:5555";
        // Socket request and response buffers
        std::stringstream request;
        char response_buffer[50] = {0};
        ArwainThread ncs2_thread;

    public:
        /** \brief Start a Python script which opens a socket and does inference on data we send it. */
        void py_inference()
        {   
            if (!arwain::config.no_inference)
            {   
                std::string command = "PYTHONPATH=/opt/intel/openvino/python/python3.7:/opt/intel/openvino/python/python3:/opt/intel/openvino/deployment_tools/model_optimizer:/opt/ros/melodic/lib/python2.7/dist-packages:/opt/intel/openvino/python/python3.7:/opt/intel/openvino/python/python3:/opt/intel/openvino/deployment_tools/model_optimizer InferenceEngine_DIR=/opt/intel/openvino/deployment_tools/inference_engine/share INTEL_OPENVINO_DIR=/opt/intel/openvino OpenCV_DIR=/opt/intel/openvino/opencv/cmake LD_LIBRARY_PATH=/opt/intel/openvino/opencv/lib:/opt/intel/openvino/deployment_tools/ngraph/lib:/opt/intel/opencl:/opt/intel/openvino/deployment_tools/inference_engine/external/hddl/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/gna/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/mkltiny_lnx/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/tbb/lib:/opt/intel/openvino/deployment_tools/inference_engine/lib/armv7l:/opt/ros/melodic/lib:/opt/intel/openvino/opencv/lib:/opt/intel/openvino/deployment_tools/ngraph/lib:/opt/intel/opencl:/opt/intel/openvino/deployment_tools/inference_engine/external/hddl/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/gna/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/mkltiny_lnx/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/tbb/lib:/opt/intel/openvino/deployment_tools/inference_engine/lib/armv7l PATH=/opt/intel/openvino/deployment_tools/model_optimizer:/opt/ros/melodic/bin:/home/pi/.vscode-server/bin/ccbaa2d27e38e5afa3e5c21c1c7bef4657064247/bin:/home/pi/.local/bin:/opt/intel/openvino/deployment_tools/model_optimizer:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games /usr/bin/python3 ./python_utils/ncs2_interface.py " + arwain::config.inference_model_xml + " > /dev/null &";
                system(command.c_str());
            }
        }

        NCS2Inferrer()
        {
            ncs2_thread = ArwainThread{&NCS2Inferrer::py_inference, "arwain_ncs2_th", this}; // Temporary: Run Python script to handle velocity inference.
        }

        Vector3 infer(const std::deque<ImuData>& imu_data)
        {
            // Load the IMU data into a string for serial transmission, gyro first.
            for (unsigned int i = 0; i < imu_data.size(); i++)
            {
                request << std::setprecision(15) << (float)imu_data[i].gyro.x << ",";
                request << std::setprecision(15) << (float)imu_data[i].gyro.y << ",";
                request << std::setprecision(15) << (float)imu_data[i].gyro.z << ",";
                request << std::setprecision(15) << (float)imu_data[i].acce.x << ",";
                request << std::setprecision(15) << (float)imu_data[i].acce.y << ",";
                request << std::setprecision(15) << (float)imu_data[i].acce.z << ",";
            }
            // Send the data and await response.
            std::string fromStream = request.str();
            const char *str = fromStream.c_str();
            zmq_send(responder, str, strlen(str), 0);
            zmq_recv(responder, response_buffer, 50, 0);
            request.str("");

            // Process the answer buffer into local velocity buffers.
            // Assume a comma-separated list of three floats.
            std::string answer{response_buffer};
            if (answer == "accept")
            {
                return {0, 0, 0};
            }

            Vector3 velocity = {0, 0, 0};
            int delimiter = answer.find(",");
            std::stringstream(answer.substr(0, delimiter)) >> velocity.x;
            answer = answer.substr(delimiter+1);
            delimiter = answer.find(",");
            std::stringstream(answer.substr(0, delimiter)) >> velocity.y;
            std::stringstream(answer.substr(delimiter+1)) >> velocity.z;
            return velocity;
        }

        bool ready = false;

        void init()
        {
            if (!ready)
            {
                context = zmq_ctx_new();
                responder = zmq_socket(context, ZMQ_REP);
                zmq_bind(responder, inference_tcp_socket.c_str());
                ready = true;
            }
        }

        ~NCS2Inferrer()
        {
            zmq_send(responder, "stop", strlen("stop"), 0);

            // Apparently deletion of a void pointer is undefined. Not sure what to do about this.
            // if (context != nullptr)
            // {
            //     delete context;
            // }
            // if (responder != nullptr)
            // {
            //     delete responder;
            // }
            if (ncs2_thread.joinable())
            {
                ncs2_thread.join();
            }
        }
};
#endif

#endif
