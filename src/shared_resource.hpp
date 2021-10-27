#ifndef _ARWAIN_SHARED_RESOURCE_HPP
#define _ARWAIN_SHARED_RESOURCE_HPP

namespace arwain
{
    class Configuration;
    extern int shutdown;
    extern std::string folder_date_string;
    extern Configuration config;
    extern arwain::Status status;
}

/** \brief Contains mutex locks for thread coordination. */
namespace arwain::Locks
{
    extern std::mutex PRESSURE_BUFFER_LOCK;
    extern std::mutex IMU_BUFFER_LOCK;
    extern std::mutex MAG_BUFFER_LOCK;
    extern std::mutex VELOCITY_BUFFER_LOCK;
    extern std::mutex STATUS_FLAG_LOCK;
    extern std::mutex POSITION_BUFFER_LOCK;
    extern std::mutex ORIENTATION_BUFFER_LOCK;
    extern std::mutex PRESSURE_BUFFER_LOCK;
}

namespace arwain::Buffers
{
    extern std::deque<vector6> IMU_BUFFER;
    extern std::deque<vector6> IMU_WORLD_BUFFER;
    extern std::deque<vector3> VELOCITY_BUFFER;
    extern std::deque<vector3> POSITION_BUFFER;
    extern std::deque<vector3> MAG_BUFFER;
    extern std::deque<vector3> MAG_WORLD_BUFFER;
    extern std::deque<vector3> IPS_BUFFER;
    extern std::deque<vector3> PRESSURE_BUFFER;
    extern std::deque<euler_orientation_t> EULER_ORIENTATION_BUFFER;
    extern std::deque<quaternion> QUAT_ORIENTATION_BUFFER;
}

#endif
