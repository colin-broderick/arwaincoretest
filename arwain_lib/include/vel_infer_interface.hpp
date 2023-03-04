#ifndef _GREEVE_VEL_INFER_INTERFACE
#define _GREEVE_VEL_INFER_INTERFACE

#include <deque>

class Vector3;
class ImuData;

class I_VelInferrer
{
    public:
        virtual void init() = 0;
        virtual Vector3 infer(const std::deque<ImuData>& imu_data) = 0;
};

#endif
