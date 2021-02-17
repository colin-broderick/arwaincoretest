#ifndef VECTOR3_H
#define VECTOR3_H

#include <math.h>

typedef struct vector3
{
    float x;
    float y;
    float z;
    float magnitude();
} vector3;

inline float vector3::magnitude()
{
    return sqrt(x*x+y*y+z*z);
}

inline vector3 operator+(vector3 v1, vector3 v2)
{
    return vector3{
        v1.x + v2.x,
        v1.y + v2.y,
        v1.z + v2.z
    };
};

inline vector3 operator-(vector3 v1, vector3 v2)
{
    return vector3{
        v1.x - v2.x,
        v1.y - v2.y,
        v1.z - v2.z
    };
};

template <class T>
inline vector3 operator/(vector3 v, T scalar)
{
    return vector3{
        v.x/scalar,
        v.y/scalar,
        v.z/scalar
    };
};

template <class T>
inline vector3 operator*(vector3 v, T scalar)
{
    return vector3{
        v.x*scalar,
        v.y*scalar,
        v.z*scalar
    };
};

#endif
