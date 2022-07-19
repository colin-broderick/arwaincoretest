#include "t265.hpp"

T265::T265()
{
    pipe = new rs2::pipeline;
    pipe->start();
}

T265::~T265()
{
    delete pipe;
}

Vector3 T265::get_position() const
{
    rs2::frameset frame_set = pipe->wait_for_frames();
    
    auto frame = frame_set.first_or_default(RS2_STREAM_POSE);
    auto pose_data = frame.as<rs2::pose_frame>().get_pose_data();
    
    return Vector3{
        pose_data.translation.x,
        pose_data.translation.y,
        pose_data.translation.z
    };
}