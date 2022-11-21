#include <cmath>

#include "activity_metric.hpp"
#include "arwain_utils.hpp"
#include "vector3.hpp"

ActivityMetric::ActivityMetric(unsigned int ag_window_size_, unsigned int velo_window_size_)
: ag_window_size(ag_window_size_), velo_window_size(velo_window_size_)
{
    acce_roller = RollingAverage{ag_window_size_};
    gyro_roller = RollingAverage{ag_window_size_};
    velo_roller = RollingAverage{velo_window_size_};
}

ActivityMetric::~ActivityMetric()
{
    
}

void ActivityMetric::feed_gyro(const Vector3& gyro)
{
    gyro_roller.feed(gyro.magnitude());
}

void ActivityMetric::feed_acce(const Vector3& acce)
{
    acce_roller.feed(acce.magnitude());
}

void ActivityMetric::feed_velo(const Vector3& velo)
{
    velo_roller.feed(velo.magnitude());
}

double ActivityMetric::read()
{
    return 4 * std::abs(
          ((acce_roller.get_value() - acce_mean) / acce_stdv)
        * ((gyro_roller.get_value() - gyro_mean) / gyro_stdv)
        / ((velo_roller.get_value() - velo_mean) / velo_stdv)
    );
}
