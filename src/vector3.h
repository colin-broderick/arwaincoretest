#ifndef VECTOR3_H
#define VECTOR3_H

typedef struct vector3
{
    float x;
    float y;
    float z;
} vector3;

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

#endif
