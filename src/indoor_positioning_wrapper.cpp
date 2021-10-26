#include "indoor_positioning_wrapper.hpp"

void arwain::IndoorPositioningWrapper::update(const double &time, const double &x, const double &y, const double &z)
{
    // TODO Pass the parameters to the inner IPS object and put results in
    // m_x, m_y, m_z.
}

vector3 arwain::IndoorPositioningWrapper::getPosition()
{
    return vector3{
        m_x,
        m_y,
        m_z
    };
}

double arwain::IndoorPositioningWrapper::getX()
{
    return m_x;
}

double arwain::IndoorPositioningWrapper::getY()
{
    return m_y;
}

double arwain::IndoorPositioningWrapper::getZ()
{
    return m_z;
}
