#include <gtest/gtest.h>

#include "lis3mdl.hpp"

TEST(LIS3MDL, Constructor_fail)
{
    // Construct with invalid bus name.
    EXPECT_THROW((LIS3MDL<I2CDEVICEDRIVER>{1, "fail_bus"}), std::runtime_error);
}

TEST(LIS3MDL, read)
{
    LIS3MDL<I2CDEVICEDRIVER> mag{1, "bus"};
    Vector3 expected{0, 0, 0};
    EXPECT_EQ(mag.read(), expected);
}

TEST(LIS3MDL, set_fsr)
{
    LIS3MDL<I2CDEVICEDRIVER> mag{1, "bus"};
    EXPECT_NO_THROW(mag.set_fsr(LIS3MDL<I2CDEVICEDRIVER>::FSR::FSR_4));
    EXPECT_NO_THROW(mag.set_fsr(LIS3MDL<I2CDEVICEDRIVER>::FSR::FSR_8));
    EXPECT_NO_THROW(mag.set_fsr(LIS3MDL<I2CDEVICEDRIVER>::FSR::FSR_12));
    EXPECT_NO_THROW(mag.set_fsr(LIS3MDL<I2CDEVICEDRIVER>::FSR::FSR_16));
}

TEST(LIS3MDL, set_odr)
{
    LIS3MDL<I2CDEVICEDRIVER> mag{1, "bus"};
    EXPECT_NO_THROW(mag.set_odr(LIS3MDL<I2CDEVICEDRIVER>::ODR::ODR_5_Hz));
    EXPECT_NO_THROW(mag.set_odr(LIS3MDL<I2CDEVICEDRIVER>::ODR::ODR_10_Hz));
    EXPECT_NO_THROW(mag.set_odr(LIS3MDL<I2CDEVICEDRIVER>::ODR::ODR_20_Hz));
    EXPECT_NO_THROW(mag.set_odr(LIS3MDL<I2CDEVICEDRIVER>::ODR::ODR_40_Hz));
    EXPECT_NO_THROW(mag.set_odr(LIS3MDL<I2CDEVICEDRIVER>::ODR::ODR_80_Hz));
    EXPECT_NO_THROW(mag.set_odr(LIS3MDL<I2CDEVICEDRIVER>::ODR::ODR_155_Hz));
    EXPECT_NO_THROW(mag.set_odr(LIS3MDL<I2CDEVICEDRIVER>::ODR::ODR_300_Hz));
    EXPECT_NO_THROW(mag.set_odr(LIS3MDL<I2CDEVICEDRIVER>::ODR::ODR_500_Hz));
    EXPECT_NO_THROW(mag.set_odr(LIS3MDL<I2CDEVICEDRIVER>::ODR::ODR_1000_Hz));
}

TEST(LIS3MDL, set_calibration_parameters)
{
    LIS3MDL<I2CDEVICEDRIVER> mag{1, "bus"};
    Vector3 stuff{1, 2, 3};
    EXPECT_NO_THROW(mag.set_calibration_parameters(stuff, stuff, stuff));
}

TEST(LIS3MDL, DefaultConstructor)
{
    EXPECT_NO_THROW(
        LIS3MDL<I2CDEVICEDRIVER> mag;
    );
}

TEST(LIS3MDL, test_chip)
{
    LIS3MDL<I2CDEVICEDRIVER> mag{1, "bus"};
    EXPECT_EQ(mag.test_chip(), 0);
}
