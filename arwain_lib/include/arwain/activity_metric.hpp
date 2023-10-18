#ifndef _ARWAIN_ACTIVITY_METRIC_HPP
#define _ARWAIN_ACTIVITY_METRIC_HPP

#include "arwain/utils.hpp"

class Vector3;

/** \brief Keeps track of an activity intensity metric based on gyro, accel, and velocity
 * data. Low intensity results for lack of motion or 'normal' motions. High intensity 
 * results from unusual motions or high sensor activity with little velocity.
*/
class ActivityMetric
{
    constexpr static double acce_mean = 10.017;
    constexpr static double acce_stdv = 2.399;
    constexpr static double gyro_mean = 0.569;
    constexpr static double gyro_stdv = 0.647;
    constexpr static double velo_mean = 0.510;  
    constexpr static double velo_stdv = 0.485;

    public:
        ActivityMetric(unsigned int ag_window_size, unsigned int velocity_window_size);
        void feed_gyro(const Vector3& acce);
        void feed_acce(const Vector3& gyro);
        void feed_velocity(const Vector3& velocity);
        double read();
    private:
        unsigned int ag_window_size;
        unsigned int velo_window_size;
        RollingAverage acce_roller;
        RollingAverage gyro_roller;
        RollingAverage velo_roller;
};

#endif
