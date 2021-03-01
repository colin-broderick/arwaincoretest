#ifndef FILTER_H
#define FILTER_H

class Filter
{
    public:
        virtual void update(double timestamp, double gx, double gy, double gz, double ax, double ay, double az){};
        virtual void update(double timestamp, double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz){};
        virtual double getW(){return 0;}
        virtual double getX(){return 0;}
        virtual double getY(){return 0;}
        virtual double getZ(){return 0;}
        virtual double getPitch(){return 0;}
        virtual double getRoll(){return 0;}
        virtual double getYaw(){return 0;}
};

#endif
 