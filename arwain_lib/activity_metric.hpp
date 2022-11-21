#ifndef _ARWAIN_ACTIVITY_METRIC_HPP
#define _ARWAIN_ACTIVITY_METRIC_HPP

class Vector3;
namespace arwain { class RollingAverage; }

class ActivityMetric
{
    constexpr static double acce_mean = 10.017;
    constexpr static double acce_stdv = 2.399;
    constexpr static double gyro_mean = 0.569;
    constexpr static double gyro_stdv = 0.647;
    constexpr static double velo_mean = 0.510;
    constexpr static double velo_stdv = 0.485;

    public:
        ActivityMetric(unsigned int ag_window_size, unsigned int velo_window_size);
        ~ActivityMetric();
        void feed_gyro(const Vector3& acce);
        void feed_acce(const Vector3& gyro);
        void feed_velo(const Vector3& velo);
        double read() const;
    private:
        unsigned int ag_window_size;
        unsigned int velo_window_size;
        arwain::RollingAverage* acce_roller;
        arwain::RollingAverage* gyro_roller;
        arwain::RollingAverage* velo_roller;
};

#endif
