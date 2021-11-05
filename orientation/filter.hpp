#ifndef FILTER_H
#define FILTER_H

#include <array>
#include <cmath>

namespace arwain
{
    /** \brief All orientation filters should extend this base class
     * You can instantiate the derived class by
     *    Filter* filter;
     *    filter = new DeriveFilter{...}
     *    delete filter;
     */
    class Filter
    {
        public:
            virtual ~Filter() = default;
            virtual void update(double timestamp, double gx, double gy, double gz, double ax, double ay, double az) = 0;
            virtual void update(double timestamp, double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz) = 0;
            virtual double getW() = 0;
            virtual double getX() = 0;
            virtual double getY() = 0;
            virtual double getZ() = 0;
            virtual void setQ(double, double, double, double) = 0;
            virtual double getPitch() = 0;
            virtual double getRoll() = 0;
            virtual double getYaw() = 0;
            static std::array<double, 3> getEulerAnglesRadians(double w, double x, double y, double z);
            static std::array<double, 3> getEulerAnglesDegrees(double w, double x, double y, double z);
    };
}

/** \brief Gives roll, pitch, yaw, in that order. */
inline std::array<double, 3> arwain::Filter::getEulerAnglesRadians(double w, double x, double y, double z)
{
    double roll = std::atan2(w*x + y*z, 0.5 - x*x - y*y);
	double pitch = std::asin(-2.0 * (x*z - w*y));
	double yaw = std::atan2(x*y + w*z, 0.5 - y*y - z*z);
    
    return std::array<double, 3>{roll, pitch, yaw};
}

/** \brief Gives roll, pitch, yaw, in that order. */
inline std::array<double, 3> arwain::Filter::getEulerAnglesDegrees(double w, double x, double y, double z)
{
    double roll = std::atan2(w*x + y*z, 0.5 - x*x - y*y)*180.0/3.14159;
	double pitch = std::asin(-2.0 * (x*z - w*y))*180.0/3.14159;
	double yaw = std::atan2(x*y + w*z, 0.5 - y*y - z*z)*180.0/3.14159;
    
    return std::array<double, 3>{roll, pitch, yaw};
}

#endif
