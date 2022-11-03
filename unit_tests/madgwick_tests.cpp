//includes
#include <cmath>

#include "madgwick.hpp"
#include "gtest/gtest.h"

TEST(Madgwick, Default_Constructor)
{
    arwain::Madgwick mad;
    
    EXPECT_TRUE(mad.get_beta() == 0.1);
    //all other parts of the constructor can't be verified due to being private with no getters.
}

TEST(Madgwick, Constructor)
{
    arwain::Madgwick mad(2.0, 1.0);
    
    EXPECT_TRUE(mad.get_beta() == 1.0);
    //sample frequency can't be verified - NEEDS TO BE VERIFIED!!!
    //all other parts of the constructor can't be verified due to being private with no getters.
}

TEST(Madgwick, Get_Beta)
{
    arwain::Madgwick mad;
    double beta = mad.get_beta();
    EXPECT_TRUE(beta == 0.1);
}