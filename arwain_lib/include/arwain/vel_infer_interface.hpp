#ifndef _GREEVE_VEL_INFER_INTERFACE
#define _GREEVE_VEL_INFER_INTERFACE

#include <deque>

class Vector3;
struct ImuData;

class I_VelInferrer
{
    public:
        virtual Vector3 infer(std::deque<ImuData>& imu_data) = 0;
};

#endif
