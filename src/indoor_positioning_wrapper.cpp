#include "indoor_positioning_wrapper.h"

void IndoorPositioningWrapper::update(const double &time, const double &x, const double &y, const double &z)
{
    // TODO Pass the parameters to the inner IPS object and put results in
    // m_x, m_y, m_z.
}

std::array<double, 3> IndoorPositioningWrapper::getPosition()
{
    return std::array<double, 3>{
        m_x,
        m_y,
        m_z
    };
}

double IndoorPositioningWrapper::getX()
{
    return m_x;
}

double IndoorPositioningWrapper::getY()
{
    return m_y;
}

double IndoorPositioningWrapper::getZ()
{
    return m_z;
}
