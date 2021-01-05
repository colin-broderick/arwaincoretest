#include <string>
#include <sstream>
#include <ctime>


typedef struct euler_orientation_t
{
    float roll;
    float pitch;
    float yaw;
} euler_orientation_t;

typedef struct quat_orientation_t
{
    float w;
    float x;
    float y;
    float z;
} quat_orientation_t;

typedef struct velocity_t
{
    float x;
    float y;
    float z;
} velocity_t;

typedef struct position_t
{
    float x;
    float y;
    float z;
} position_t;

// Date/time as a string in the form YYYY_MM_DD_HH_mm_ss.
std::string datetimestring();