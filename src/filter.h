#ifndef FILTER_H
#define FILTER_H

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
        virtual double getPitch() = 0;
        virtual double getRoll() = 0;
        virtual double getYaw() = 0;
};

#endif
