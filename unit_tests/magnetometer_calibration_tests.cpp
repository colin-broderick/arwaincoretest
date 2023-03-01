#include <gtest/gtest.h>
#include <chrono>

#include "vector3.hpp"
#include "calibration.hpp"

/** \brief Until the 100th feed, the feed_count reflects the number of feeds so far. */
TEST(Calibration, MagnetometerCalibrator_Feed_less_than)
{
   MagnetometerCalibrator mag_cal;
   Vector3 reading{1, 1, 1};

   for (int i = 0; i < 99; i++)
   {
      mag_cal.feed(reading);
      EXPECT_EQ(mag_cal.get_feed_count(), i + 1);
   }
}

/** \brief On the 100th feed, the calibrator increments feed_count an extra
 * time for internal reasons, hence the 101 instead of 100. This is probably
 * confusing and should be improved. TODO
 */
TEST(Calibration, MagnetometerCalibrator_Feed_more_than)
{
   MagnetometerCalibrator mag_cal;
   Vector3 reading{1, 1, 1};

   for (int i = 0; i <= 100; i++)
   {
      mag_cal.feed(reading);
   }
   EXPECT_EQ(mag_cal.get_feed_count(), 101);
}

TEST(Calibration, MagnetometerCalibrator_get_sphere_coverage_quality)
{
   MagnetometerCalibrator mag_cal;
   EXPECT_EQ(mag_cal.get_sphere_coverage_quality(), 0);
   mag_cal.sphere_coverage_quality = 17;
   EXPECT_EQ(mag_cal.get_sphere_coverage_quality(), 17);
}

TEST(Calibration, MagnetometerCalibrator_sphere_coverage_0)
{
   MagnetometerCalibrator mag_cal;
   std::array<int, 100> region_sample_count;
   for (unsigned int i = 0; i < region_sample_count.size(); i++)
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
   for (unsigned int i = 0; i < region_sample_count.size(); i++)
   {
      region_sample_count[i] = 1;
   }
   int coverage = mag_cal.sphere_coverage(region_sample_count);
   EXPECT_TRUE(coverage == 100);
}

TEST(Calibration, MagnetometerCalibrator_get_feed_count)
{
   MagnetometerCalibrator mag_cal;
   int count = mag_cal.get_feed_count();
   EXPECT_TRUE(count == 0);
}

TEST(Calibration, MagnetometerCalibrator_get_region_sample_count)
{
   MagnetometerCalibrator mag_cal;
   std::array<int, 100> rsc = mag_cal.get_region_sample_count();
   std::array<int, 100> comparison;
   for (unsigned int i = 0; i < comparison.size(); i++)
   {
      comparison[i] = 0;
   }
   EXPECT_EQ(comparison, rsc);
}

TEST(Calibration, MagnetometerCalibrator_get_region_sample_value)
{
   MagnetometerCalibrator mag_cal;
   std::array<Vector3, 100> rsv = mag_cal.get_region_sample_value();
   std::array<Vector3, 100> comparison;
   for (unsigned int i = 0; i < comparison.size(); i++)
   {
      comparison[i].x = 0;
      comparison[i].y = 0;
      comparison[i].z = 0;
   }
   EXPECT_EQ(comparison, rsv);
}

/** \brief Can't be effectively tested without sample data. */
TEST(NOTREADY_Calibration, MagnetometerCalibrator_Solve)
{
   FAIL();
}

TEST(Calibration, MagnetometerCalibrator_sphere_region)
{
   MagnetometerCalibrator mag_cal;

   // region 0
   int region = mag_cal.sphere_region(0, 0, 1);
   EXPECT_TRUE(region == 0);

   // region on boundry test
   region = mag_cal.sphere_region(-5, 0, 5);
   EXPECT_TRUE(region == 15);

   // 1
   region = mag_cal.sphere_region(-4.56773, -2.03368, 5);
   EXPECT_TRUE(region == 1);

   // 2
   region = mag_cal.sphere_region(-3.34566, -3.71572, 5);
   EXPECT_TRUE(region == 2);

   // 3
   region = mag_cal.sphere_region(-1.54509, -4.75528, 5);
   EXPECT_TRUE(region == 3);

   // 4
   region = mag_cal.sphere_region(0.522635, -4.97261, 5);
   EXPECT_TRUE(region == 4);

   // 5
   region = mag_cal.sphere_region(2.49999, -4.33013, 5);
   EXPECT_TRUE(region == 5);

   // 6
   region = mag_cal.sphere_region(4.04508, -2.93893, 5);
   EXPECT_TRUE(region == 6);

   // 7
   region = mag_cal.sphere_region(4.89074, -1.03957, 5);
   EXPECT_TRUE(region == 7);

   // 8
   region = mag_cal.sphere_region(4.89074, 1.03954, 5);
   EXPECT_TRUE(region == 8);

   // 9
   region = mag_cal.sphere_region(4.04509, 2.93891, 5);
   EXPECT_TRUE(region == 9);

   // 10
   region = mag_cal.sphere_region(2.50002, 4.33012, 5);
   EXPECT_TRUE(region == 10);

   // 11
   region = mag_cal.sphere_region(0.522662, 4.97261, 5);
   EXPECT_TRUE(region == 11);

   // 12
   region = mag_cal.sphere_region(-1.54506, 4.75529, 5);
   EXPECT_TRUE(region == 12);

   // 13
   region = mag_cal.sphere_region(-3.34564, 3.71574, 5);
   EXPECT_TRUE(region == 13);

   // 14
   region = mag_cal.sphere_region(-4.56772, 2.03371, 5);
   EXPECT_TRUE(region == 14);

   // 15
   region = mag_cal.sphere_region(-5, 2.65359e-05, 5);
   EXPECT_TRUE(region == 15);

   // 16
   region = mag_cal.sphere_region(-4.91487, -0.918747, 2);
   EXPECT_TRUE(region == 16);

   // 17
   region = mag_cal.sphere_region(-4.66236171089638, -1.80620687540698, 2);
   EXPECT_TRUE(region == 17);

   // 18
   region = mag_cal.sphere_region(-4.25109, -2.63216, 2);
   EXPECT_TRUE(region == 18);

   // 19
   region = mag_cal.sphere_region(-3.69505, -3.36848, 2);
   EXPECT_TRUE(region == 19);

   // 20
   region = mag_cal.sphere_region(-3.01318, -3.99008, 2);
   EXPECT_TRUE(region == 20);

   // 21
   region = mag_cal.sphere_region(-2.2287, -4.47581, 2);
   EXPECT_TRUE(region == 21);

   // 22
   region = mag_cal.sphere_region(-1.36832, -4.80913, 2);
   EXPECT_TRUE(region == 22);

   // 23
   region = mag_cal.sphere_region(-0.461348, -4.97867, 2);
   EXPECT_TRUE(region == 23);

   // 24
   region = mag_cal.sphere_region(0.461335, -4.97867, 2);
   EXPECT_TRUE(region == 24);

   // 25
   region = mag_cal.sphere_region(1.36831, -4.80913, 2);
   EXPECT_TRUE(region == 25);

   // 26
   region = mag_cal.sphere_region(2.22868, -4.47582, 2);
   EXPECT_TRUE(region == 26);

   // 27
   region = mag_cal.sphere_region(3.01317, -3.99009, 2);
   EXPECT_TRUE(region == 27);

   // 28
   region = mag_cal.sphere_region(3.69504, -3.36849, 2);
   EXPECT_TRUE(region == 28);

   // 29
   region = mag_cal.sphere_region(4.25108, -2.63217, 2);
   EXPECT_TRUE(region == 29);

   // 30
   region = mag_cal.sphere_region(4.66236, -1.80622, 2);
   EXPECT_TRUE(region == 30);

   // 31
   region = mag_cal.sphere_region(4.91486, -0.91876, 2);
   EXPECT_TRUE(region == 31);

   // 32
   region = mag_cal.sphere_region(5, -1.32679e-05, 2);
   EXPECT_TRUE(region == 32);

   // 33
   region = mag_cal.sphere_region(4.91487, 0.918734, 2);
   EXPECT_TRUE(region == 33);

   // 34
   region = mag_cal.sphere_region(4.66237, 1.80619, 2);
   EXPECT_TRUE(region == 34);

   // 35
   region = mag_cal.sphere_region(4.25109, 2.63215, 2);
   EXPECT_TRUE(region == 35);

   // 36
   region = mag_cal.sphere_region(3.69506, 3.36847, 2);
   EXPECT_TRUE(region == 36);

   // 37
   region = mag_cal.sphere_region(3.01319, 3.99008, 2);
   EXPECT_TRUE(region == 37);

   // 38
   region = mag_cal.sphere_region(2.22871, 4.47581, 2);
   EXPECT_TRUE(region == 38);

   // 39
   region = mag_cal.sphere_region(1.36833, 4.80912, 2);
   EXPECT_TRUE(region == 39);

   // 40
   region = mag_cal.sphere_region(0.461361, 4.97867, 2);
   EXPECT_TRUE(region == 40);

   // 41
   region = mag_cal.sphere_region(-0.461322, 4.97867, 2);
   EXPECT_TRUE(region == 41);

   // 42
   region = mag_cal.sphere_region(-1.36829, 4.80913, 2);
   EXPECT_TRUE(region == 42);

   // 43
   region = mag_cal.sphere_region(-2.22867, 4.47583, 2);
   EXPECT_TRUE(region == 43);

   // 44
   region = mag_cal.sphere_region(-3.01316, 3.9901, 2);
   EXPECT_TRUE(region == 44);

   // 45
   region = mag_cal.sphere_region(-3.69503, 3.3685, 2);
   EXPECT_TRUE(region == 45);

   // 46
   region = mag_cal.sphere_region(-4.25107, 2.63218, 2);
   EXPECT_TRUE(region == 46);

   // 47
   region = mag_cal.sphere_region(-4.66235, 1.80623, 2);
   EXPECT_TRUE(region == 47);

   // 48
   region = mag_cal.sphere_region(-4.91486, 0.918773, 2);
   EXPECT_TRUE(region == 48);

   // 49
   region = mag_cal.sphere_region(-5, 2.65359e-05, 2);
   EXPECT_TRUE(region == 49);

   // 50
   region = mag_cal.sphere_region(-4.91487, -0.918747, -2);
   EXPECT_TRUE(region == 50);

   // 51
   region = mag_cal.sphere_region(-4.66236171089638, -1.80620687540698, -2);
   EXPECT_TRUE(region == 51);

   // 52
   region = mag_cal.sphere_region(-4.25109, -2.63216, -2);
   EXPECT_TRUE(region == 52);

   // 53
   region = mag_cal.sphere_region(-3.69505, -3.36848, -2);
   EXPECT_TRUE(region == 53);

   // 54
   region = mag_cal.sphere_region(-3.01318, -3.99008, -2);
   EXPECT_TRUE(region == 54);

   // 55
   region = mag_cal.sphere_region(-2.2287, -4.47581, -2);
   EXPECT_TRUE(region == 55);

   // 56
   region = mag_cal.sphere_region(-1.36832, -4.80913, -2);
   EXPECT_TRUE(region == 56);

   // 57
   region = mag_cal.sphere_region(-0.461348, -4.97867, -2);
   EXPECT_TRUE(region == 57);

   // 58
   region = mag_cal.sphere_region(0.461335, -4.97867, -2);
   EXPECT_TRUE(region == 58);

   // 59
   region = mag_cal.sphere_region(1.36831, -4.80913, -2);
   EXPECT_TRUE(region == 59);

   // 60
   region = mag_cal.sphere_region(2.22868, -4.47582, -2);
   EXPECT_TRUE(region == 60);

   // 61
   region = mag_cal.sphere_region(3.01317, -3.99009, -2);
   EXPECT_TRUE(region == 61);

   // 62
   region = mag_cal.sphere_region(3.69504, -3.36849, -2);
   EXPECT_TRUE(region == 62);

   // 63
   region = mag_cal.sphere_region(4.25108, -2.63217, -2);
   EXPECT_TRUE(region == 63);

   // 64
   region = mag_cal.sphere_region(4.66236, -1.80622, -2);
   EXPECT_TRUE(region == 64);

   // 65
   region = mag_cal.sphere_region(4.91486, -0.91876, -2);
   EXPECT_TRUE(region == 65);

   // 66
   region = mag_cal.sphere_region(5, -1.32679e-05, -2);
   EXPECT_TRUE(region == 66);

   // 67
   region = mag_cal.sphere_region(4.91487, 0.918734, -2);
   EXPECT_TRUE(region == 67);

   // 68
   region = mag_cal.sphere_region(4.66237, 1.80619, -2);
   EXPECT_TRUE(region == 68);

   // 69
   region = mag_cal.sphere_region(4.25109, 2.63215, -2);
   EXPECT_TRUE(region == 69);

   // 70
   region = mag_cal.sphere_region(3.69506, 3.36847, -2);
   EXPECT_TRUE(region == 70);

   // 71
   region = mag_cal.sphere_region(3.01319, 3.99008, -2);
   EXPECT_TRUE(region == 71);

   // 72
   region = mag_cal.sphere_region(2.22871, 4.47581, -2);
   EXPECT_TRUE(region == 72);

   // 73
   region = mag_cal.sphere_region(1.36833, 4.80912, -2);
   EXPECT_TRUE(region == 73);

   // 74
   region = mag_cal.sphere_region(0.461361, 4.97867, -2);
   EXPECT_TRUE(region == 74);

   // 75
   region = mag_cal.sphere_region(-0.461322, 4.97867, -2);
   EXPECT_TRUE(region == 75);

   // 76
   region = mag_cal.sphere_region(-1.36829, 4.80913, -2);
   EXPECT_TRUE(region == 76);

   // 77
   region = mag_cal.sphere_region(-2.22867, 4.47583, -2);
   EXPECT_TRUE(region == 77);

   // 78
   region = mag_cal.sphere_region(-3.01316, 3.9901, -2);
   EXPECT_TRUE(region == 78);

   // 79
   region = mag_cal.sphere_region(-3.69503, 3.3685, -2);
   EXPECT_TRUE(region == 79);

   // 80
   region = mag_cal.sphere_region(-4.25107, 2.63218, -2);
   EXPECT_TRUE(region == 80);

   // 81
   region = mag_cal.sphere_region(-4.66235, 1.80623, -2);
   EXPECT_TRUE(region == 81);

   // 82
   region = mag_cal.sphere_region(-4.91486, 0.918773, -2);
   EXPECT_TRUE(region == 82);

   // 83
   region = mag_cal.sphere_region(-5, 2.65359e-05, -2);
   EXPECT_TRUE(region == 83);

   // 84
   region = mag_cal.sphere_region(-4.56773, -2.03368, -5);
   EXPECT_TRUE(region == 84);

   // 85
   region = mag_cal.sphere_region(-3.34566, -3.71572, -5);
   EXPECT_TRUE(region == 85);

   // 86
   region = mag_cal.sphere_region(-1.54509, -4.75528, -5);
   EXPECT_TRUE(region == 86);

   // 87
   region = mag_cal.sphere_region(0.522635, -4.97261, -5);
   EXPECT_TRUE(region == 87);

   // 88
   region = mag_cal.sphere_region(2.49999, -4.33013, -5);
   EXPECT_TRUE(region == 88);

   // 89
   region = mag_cal.sphere_region(4.04508, -2.93893, -5);
   EXPECT_TRUE(region == 89);

   // 90
   region = mag_cal.sphere_region(4.89074, -1.03957, -5);
   EXPECT_TRUE(region == 90);

   // 91
   region = mag_cal.sphere_region(4.89074, 1.03954, -5);
   EXPECT_TRUE(region == 91);

   // 92
   region = mag_cal.sphere_region(4.04509, 2.93891, -5);
   EXPECT_TRUE(region == 92);

   // 93
   region = mag_cal.sphere_region(2.50002, 4.33012, -5);
   EXPECT_TRUE(region == 93);

   // 94
   region = mag_cal.sphere_region(0.522662, 4.97261, -5);
   EXPECT_TRUE(region == 94);

   // 95
   region = mag_cal.sphere_region(-1.54506, 4.75529, -5);
   EXPECT_TRUE(region == 95);

   // 96
   region = mag_cal.sphere_region(-3.34564, 3.71574, -5);
   EXPECT_TRUE(region == 96);

   // 97
   region = mag_cal.sphere_region(-4.56772, 2.03371, -5);
   EXPECT_TRUE(region == 97);

   // 98
   region = mag_cal.sphere_region(-5, 2.65359e-05, -5);
   EXPECT_TRUE(region == 98);

   // 99
   region = mag_cal.sphere_region(0, 0, -1);
   EXPECT_TRUE(region == 99);
}
