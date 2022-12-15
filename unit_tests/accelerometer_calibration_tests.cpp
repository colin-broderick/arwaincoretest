#include <gtest/gtest.h>
#include <tuple>

#include "vector3.hpp"
#include "calibration.hpp"

TEST(Calibration, AccelerometerCalibrator_is_converged)
{
   AccelerometerCalibrator acc_cal;
   EXPECT_FALSE(acc_cal.is_converged());
}

TEST(Calibration, AccelerometerCalibrator_get_params)
{
   AccelerometerCalibrator acc_cal;
   Vector3 comparison = {9.81, 9.81, 9.81};
   EXPECT_TRUE(acc_cal.get_params() == comparison);
}

TEST(Calibration, AccelerometerCalibrator_next_sampling)
{
   AccelerometerCalibrator acc_cal;

   // check size of samplings with local variable
   std::vector<Vector3> sample = acc_cal.get_samplings();

   acc_cal.next_sampling();
   EXPECT_NE(sample.size(), acc_cal.get_samplings().size());
   EXPECT_FALSE(acc_cal.is_converged());
   Vector3 comparison = {9.81, 9.81, 9.81};
   EXPECT_TRUE(acc_cal.get_params() == comparison);
}

TEST(Calibration, AccelerometerCalibrator_feed_false)
{
   AccelerometerCalibrator acc_cal;
   Vector3 reading = {1.0, 1.0, 1.0};
   bool converged = acc_cal.feed(reading);
   EXPECT_FALSE(converged);
}

TEST(Calibration, AccelerometerCalibrator_feed_true)
{
   AccelerometerCalibrator acc_cal;
   Vector3 reading = {0.0, 0.0, 9.81};

   for (int i = 0; i < 10000; i++)
   {
      acc_cal.feed(reading);
   }

   EXPECT_TRUE(acc_cal.feed(reading));
}

TEST(Calibration, AccelerometerCalibrator_deduce_calib_params)
{
   AccelerometerCalibrator acc_cal;
   acc_cal.samplings.push_back({2.00, 2.00, 2.00});
   acc_cal.samplings.push_back({1.00, 1.00, 1.00});

   std::tuple<Vector3, Vector3> params = acc_cal.deduce_calib_params();
   Vector3 bias_comparison = {1.5, 1.5, 1.5};
   Vector3 scale_comparison = {1.00, 1.00, 1.00};
   std::tuple<Vector3, Vector3> comparison = {bias_comparison, scale_comparison};

   EXPECT_EQ(comparison, params);
}
