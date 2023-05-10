#include <gtest/gtest.h>
#include <chrono>
#include <tuple>

#include "arwain/calibration.hpp"

#include "vector3.hpp"

TEST(GyroscopeCalibrator, is_converged)
{
   GyroscopeCalibrator gyro_cal;
   EXPECT_FALSE(gyro_cal.is_converged());
}

TEST(GyroscopeCalibrator, get_params)
{
   GyroscopeCalibrator gyro_cal;
   Vector3 comparison = {0, 0, 0};
   EXPECT_EQ(gyro_cal.get_params(), comparison);
}

TEST(GyroscopeCalibrator, feed__false)
{
   GyroscopeCalibrator gyro_cal;
   Vector3 reading = {0, 0, 0};

   EXPECT_FALSE(gyro_cal.feed(reading));
}

TEST(GyroscopeCalibrator, feed__true)
{
   GyroscopeCalibrator gyro_cal;
   Vector3 reading = {0.0, 0.0, 0.0};

   for (int i = 0; i < 10000; i++)
   {
      gyro_cal.feed(reading);
   }

   EXPECT_TRUE(gyro_cal.feed(reading));
}
