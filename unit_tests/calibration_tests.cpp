#include <gtest/gtest.h>
#include <chrono>

#include "vector3.hpp"
#include "calibration.hpp"

TEST(Calibration, MagnetometerCalibrator_Feed_less_than)
{
    MagnetometerCalibrator mag_cal;
    Vector3 reading{1,1,1};

   //CANT'T CHECK ANY OF THE STATES THAT CHANGE
    mag_cal.feed(reading);

   FAIL();
}

TEST(Calibration, MagnetometerCalibrator_Feed_more_than)
{
   MagnetometerCalibrator mag_cal;
   Vector3 reading{1,1,1};

   //CANT'T CHECK ANY OF THE STATES THAT CHANGE
   for(int i = 0; i <= 100; i++)
   {
    mag_cal.feed(reading);
   }
   int quality = mag_cal.get_sphere_coverage_quality();
   FAIL();
}

TEST(Calibration, MagnetometerCalibrator_Feed_equals)
{
   MagnetometerCalibrator mag_cal;
   Vector3 reading{1,1,1};

   //CANT'T CHECK ANY OF THE STATES THAT CHANGE
   for(int i = 0; i <= 100; i++)
   {
    mag_cal.feed(reading);
   }
   int quality = mag_cal.get_sphere_coverage_quality();
   FAIL();
}

TEST(Calibration, MagnetometerCalibrator_Solve)
{
   FAIL();
}

TEST(Calibration, MagnetometerCalibrator_get_sphere_coverage_quality)
{
    MagnetometerCalibrator mag_cal;
    EXPECT_EQ(mag_cal.get_sphere_coverage_quality(),0);
}

TEST(Calibration, MagnetometerCalibrator_sphere_coverage_0)
{
   MagnetometerCalibrator mag_cal;
   std::array<int, 100> region_sample_count;
   for(int i = 0 ; i < region_sample_count.size() ; i++)
   {
         region_sample_count[i] = 0;
   }
   int coverage = mag_cal.sphere_coverage(region_sample_count);
   EXPECT_TRUE(coverage == 0);
}

TEST(Calibration, MagnetometerCalibrator_sphere_coverage_1)
{
   MagnetometerCalibrator mag_cal;
   std::array<int, 100> region_sample_count;
   for(int i = 0 ; i < region_sample_count.size() ; i++)
   {
         region_sample_count[i] = 1;
   }
   int coverage = mag_cal.sphere_coverage(region_sample_count);
   EXPECT_TRUE(coverage == 100);
}

TEST(Calibration, MagnetometerCalibrator_sphere_region)
{
   FAIL();
}

TEST(Calibration, AccelerometerCalibrator_is_converged)
{
   FAIL();
}

TEST(Calibration, AccelerometerCalibrator_get_params)
{
   FAIL();
}

TEST(Calibration, AccelerometerCalibrator_next_sampling)
{
   FAIL();
}

TEST(Calibration, AccelerometerCalibrator_feed)
{
   FAIL();
}

TEST(Calibration, AccelerometerCalibrator_deduce_calib_params)
{
   FAIL();
}

TEST(Calibration, GyroscopeCalibrator_is_converged)
{
   FAIL();
}

TEST(Calibration, GyroscopeCalibrator_get_params)
{
   FAIL();
}

TEST(Calibration, GyroscopeCalibrator_feed)
{
   FAIL();
}







