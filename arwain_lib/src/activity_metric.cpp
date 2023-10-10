#include <cmath>

#include <arwain/vector3.hpp>

#include "arwain/activity_metric.hpp"
#include "arwain/utils.hpp"

/** \brief Constructor. 
 * \param ag_window_size_ How many accelerometer and gyro readings over which to calculate the
 * rolling average acceleration and gyration.
 * \param velo_window_size_ How many velocity readings over which to calcualte the rolling average
 * velocity.
*/
ActivityMetric::ActivityMetric(unsigned int ag_window_size_, unsigned int velo_window_size_)
: ag_window_size(ag_window_size_),
  velo_window_size(velo_window_size_),
  acce_roller(RollingAverage{ag_window_size_}),
  gyro_roller(RollingAverage{ag_window_size_}),
  velo_roller(RollingAverage{velo_window_size_})
{

}

/** \brief Update the activity metric with new gyroscope data.
 * \param gyro A Vector3 of gyro data in the order x, y, z.
 */
void ActivityMetric::feed_gyro(const Vector3& gyro)
{
    gyro_roller.feed(gyro.magnitude());
}

/** \brief Update the activity metric with new accelerometer data.
 * \param gyro A Vector3 of accelerometer data in the order x, y, z.
 */
void ActivityMetric::feed_acce(const Vector3& acce)
{
    acce_roller.feed(acce.magnitude());
}

/** \brief Update the activity metric with new world-frame velocity data.
 * \param velocity A Vector3 of world-frame velocity data in the order x, y, z.
 */
void ActivityMetric::feed_velocity(const Vector3& velocity)
{
    velo_roller.feed(velocity.magnitude());
}

/** \brief Read out the current value of the activity metric.
 * \return Number representing activity intensity.
 */
double ActivityMetric::read()
{
    return 4.0 * std::abs(
          ((acce_roller.get_value() - acce_mean) / acce_stdv)
        * ((gyro_roller.get_value() - gyro_mean) / gyro_stdv)
        / ((velo_roller.get_value() - velo_mean) / velo_stdv)
    );
}
