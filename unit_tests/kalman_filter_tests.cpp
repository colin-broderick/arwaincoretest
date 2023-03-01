#include <gtest/gtest.h>

#include "kalman.hpp"

TEST(KalmanFilter, kalman_gain)
{
    MatrixXd observation(2,1);

    //parameter matices
    MatrixXd state_matrix(2, 1);
    MatrixXd blank_2d(2,2);
    MatrixXd P(2, 2);
    MatrixXd U(1, 1); //model/ predicted change - acceleration in this case
    MatrixXd A(2, 2);
    MatrixXd m(2, 1); //error  
    MatrixXd r(2, 2); //sensor noice covariance matrix
    MatrixXd w(2, 1); //error - predicted state noice matrix 
    MatrixXd q(2, 2); //error - process noice covariace matrix 

    //initial values:
    //errors - to be determined
    w(0, 0) = 0;
    w(1, 0) = 0;

    q(0, 0) = 1;
    q(1, 0) = 0;
    q(0, 1) = 0;
    q(1, 1) = 1;

    m(0, 0) = 1;
    m(1, 0) = 1;

    state_matrix(0, 0) = 0; 

    state_matrix(1, 0) = 0; 
    U(0, 0) = 0.01;
    
    A(0, 0) = 1;
    A(1, 0) = 0;
    A(0, 1) = 0.1; //delta T
    A(1, 1) = 1;

    //ERROR
    P(0, 0) = 1;
    P(1, 0) = 0;
    P(0, 1) = 0;
    P(1, 1) = 1;

    blank_2d(0, 0) = 1;
    blank_2d(1, 0) = 0;
    blank_2d(0, 1) = 0;
    blank_2d(1, 1) = 1;

    //error
    r(0, 0) = 1;
    r(1, 0) = 0;
    r(0, 1) = 0;
    r(1, 1) = 1;

    KalmanFilter filter(state_matrix, P, 0.1, U, A, blank_2d, m, r, blank_2d, blank_2d, w, q); //parameters to determine
    observation(0,0) = 5;
    observation(1,0) = 1;
    filter.kalman_one_cycle(observation, U);
    // EXPECT_NO_THROW(filter.kalman_gain());
    // filter.new_state_estimate();
    // filter.new_predicted_state();
    // filter.new_state_covariance_matrix();
}

TEST(KalmanFilter1D, constructors)
{
    KalmanFilter1D kf1;
    KalmanFilter1D kf2{1, 1};
    kf2.update(1, 1);
    kf2.update_gain(1);
    kf2.update_estimate(1);
    kf2.update_estimate_error();
    kf2.get_gain();
    SUCCEED();
}

TEST(KalmanFilter1D, converged)
{
    KalmanFilter1D kf;
    kf.converged = true;
    kf.update(1, 1);
    kf.KG = 0.001;
    kf.update_estimate(1);
}