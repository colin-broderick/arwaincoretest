#include <gtest/gtest.h>

#include "sabatini_altimeter.hpp"

TEST(SabatiniAltimeter, Constructors)
{
    EXPECT_NO_THROW(arwain::Filters::SabatiniAltimeter{});
    EXPECT_NO_THROW((arwain::Filters::SabatiniAltimeter{1, 1, 1, 1, 1}));
    
    arwain::Filters::SabatiniAltimeter sab{1, 1, 1, 1, 1};
    Eigen::Matrix<double, 2, 1> X = Eigen::Matrix<double, 2, 1>{1, 1};
    Eigen::Matrix<double, 2, 2> A = Eigen::Matrix<double, 2, 2>{{1, 1}, {0, 1}};
    Eigen::Matrix<double, 2, 2> B = Eigen::Matrix<double, 2, 2>{{1.0, 0.5}, {0.0, 1.0}};
    Eigen::Matrix<double, 2, 1> C = Eigen::Matrix<double, 2, 1>{0.5, 1.0};
    Eigen::Matrix<double, 2, 1> K = Eigen::Matrix<double, 2, 1>{std::sqrt(2), 1};

    EXPECT_EQ(sab.X, X);
    EXPECT_EQ(sab.A, A);
    EXPECT_EQ(sab.B, B);
    EXPECT_EQ(sab.C, C);
    EXPECT_EQ(sab.K, K);
}

TEST(SabatiniAltimeter, update)
{
    arwain::Filters::SabatiniAltimeter sab{1, 1, 1, 1, 1};
    double pre_position = sab.get_position();
    sab.update(1, 1);
    double post_position = sab.get_position();
    EXPECT_NE(pre_position, post_position);
}