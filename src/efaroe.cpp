#include "efaroe.h"
#include <iomanip>

std::array<float, 3> cross(std::array<float, 3> a, std::array<float, 3> b);

eFaroe::eFaroe(quaternion initial_quaternion, std::array<float, 3> gyro_bias, float gyro_error, int use_mag)
{
    gyro_bias = {0, 0, 0};
    gyro_error = 0.05;
    uk_dip = -67*3.14159265/180.0;
    emf = {cos(uk_dip), sin(uk_dip)};
    zeta = sqrt(3) * 0.01;
    last_read = 0;
    use_mag = use_mag;
    
    if (initial_quaternion == quaternion{1, 0, 0, 0})
    {
        q = {1, 0, 0, 0};
        gyro_error = 100;
        true_error = gyro_error;
        beta = sqrt(3) * gyro_error;
        conv_count = 100;
    }
    else
    {
        q = initial_quaternion;
        gyro_error = gyro_error;
        true_error = gyro_error;
        beta = sqrt(3) * gyro_error;
        conv_count = -1;
    }
}

void eFaroe::update(unsigned long timestamp, float ax, float ay, float az, float gx, float gy, float gz)
{
    float dt;

    if (conv_count > 0)
    {
        conv_count--;
        if (conv_count == 0)
        {
            std::cout << "efaroe converged" << "\n";
            gyro_error = true_error;
            beta = sqrt(3) * gyro_error;
        }
    }

    // std::cout << std::setprecision(10) << last_read << "," << timestamp << "\n";

    std::array<float, 3> acc = {ax, ay, az};
    std::array<float, 3> gyr = {gx, gy, gz};

    if (last_read == 0)
    {
        // Confirm timestamp has usable type/value
        last_read = timestamp;
        return;
    }
    else
    {
        dt = timestamp - last_read;
        if (dt > 1)
        {
            dt = 1;
        }
        last_read = timestamp;
    }
    
    // Normalize acceleration.
    float a_norm = 1.0/sqrt(ax*ax+ay*ay+az*az);
    ax = ax*a_norm;
    ay = ay*a_norm;
    az = az*a_norm;
    
    // Construct Jacobian.
    std::array<float, 3> jac_a{
        q.x*2.0*q.z - q.w*2.0*q.y,
        q.w*2.0*q.x + q.y*2.0*q.z,
        1.0 - 2.0*q.x*q.x - 2.0*q.y*q.y
    };

    // Calculate gradient.
    std::array<float, 3> grad = cross(jac_a, acc);

    // Normalize gradient.
    float grad_norm = 1.0/sqrt(grad[0]*grad[0]+grad[1]*grad[1]+grad[2]*grad[2]);
    grad[0] = grad[0]*grad_norm;
    grad[1] = grad[0]*grad_norm;
    grad[2] = grad[0]*grad_norm;

    // TODO Calculate new gyro_bias?
    std::array<float, 3> g_b;
    g_b[0] = gyro_bias[0] + grad[0]*dt*zeta;
    g_b[1] = gyro_bias[1] + grad[1]*dt*zeta;
    g_b[2] = gyro_bias[2] + grad[2]*dt*zeta;

    // Subtract gyro bias.
    std::array<float, 3> gyro;
    gyro[0] = gyr[0] - g_b[0];
    gyro[1] = gyr[1] - g_b[1];
    gyro[2] = gyr[2] - g_b[2];
    
    // TODO What is this?
    std::array<float, 3> a_v;
    a_v[0] = dt*(gyro[0]-beta*grad[0]);
    a_v[1] = dt*(gyro[1]-beta*grad[1]);
    a_v[2] = dt*(gyro[2]-beta*grad[2]);
    quaternion qav{0, a_v[0], a_v[1], a_v[2]};

    // Calculate delta orientation quaternion.
    quaternion dq{
        0.5*q.w*qav.w,
        0.5*q.x*qav.x,
        0.5*q.y*qav.y,
        0.5*q.z*qav.z
    };
    
    dq = q*qav*0.5;
    q = (q + dq).unit();
}

// Computes vectors cross product of two 3-vectors.
std::array<float, 3> cross(std::array<float, 3> a, std::array<float, 3> b)
{
    std::array<float, 3> ret{
          a[1]*b[2] - b[1]*a[2],
        -(a[0]*b[2] - b[0]*a[2]),
          a[0]*b[1] - b[0]*a[1],
    };
    return ret;
}


float eFaroe::getW()
{
    return q.w;
}

float eFaroe::getX()
{
    return q.x;
}

float eFaroe::getY()
{
    return q.y;
}

float eFaroe::getZ()
{
    return q.z;
}

float eFaroe::getPitch()
{
    return pitch;
}

float eFaroe::getYaw()
{
    return yaw;
}

float eFaroe::getRoll()
{
    return roll;
}

quaternion eFaroe::getQuat()
{
    return q;
}

std::array<float, 3> eFaroe::getEuler()
{
    roll = atan2f(q.w*q.x + q.y*q.z, 0.5f - q.x*q.x - q.y*q.y);
    pitch = asinf(-2.0f * (q.x*q.z - q.w*q.y));
    yaw = atan2f(q.x*q.y + q.w*q.z, 0.5f - q.y*q.y - q.z*q.z);
    return std::array<float, 3>{pitch, yaw, roll};
}
