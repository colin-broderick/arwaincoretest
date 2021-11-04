//includes
#include "kalman.hpp"

//namespace
using Eigen::MatrixXd;

//kalman filter class functions
void kalman_filter::kalman_gain()
{
    MatrixXd H_transpose = H.transpose();
    MatrixXd temp1;
    MatrixXd temp2;

    temp1 = (state_covariance_matrix * H_transpose);
    temp2 = ((H * state_covariance_matrix * H_transpose) + measurement_covariance_matrix);

    KG = temp1.cwiseQuotient(temp2);

    //might not always be true - to check
    KG(1, 0) = 0;
    KG(0, 1) = 0;
}

//produces the new estimate value of what ever is being measured
void kalman_filter::new_state_estimate()
{
    state_matrix = state_matrix + (KG * (Y - (H * state_matrix)));
}
void kalman_filter::new_estimate_error()
{
    state_covariance_matrix = (I - (KG * H)) * state_covariance_matrix;
}

//sets up the B matrix  - might need to change for a more complex filter(> 2-D)
void kalman_filter::set_B(double dt)
{
    B(0, 0) = pow(dt, 2.00);
    B(0, 0) = B(0, 0) * 0.5;
    B(1, 0) = dt;
}

//Equation: X = AX +BU +W
void kalman_filter::new_predicted_state()
{
    state_matrix = (A * state_matrix) + (B * control_variable_matrix) + predicted_state_noise_matrix;
}

void kalman_filter::new_state_covariance_matrix()
{
    MatrixXd A_transpose = A.transpose();
    state_covariance_matrix = (A * state_covariance_matrix * A_transpose) + process_noise_covariancce_matrix;
}

void kalman_filter::measurement_matrix_prep()
{
    Y = (C * Y) + measurement_noice;
}

MatrixXd kalman_filter::kalman_one_cycle(MatrixXd observation, MatrixXd U)
{
    control_variable_matrix = U;
    Y = observation;

    new_predicted_state();
    new_state_covariance_matrix();
    kalman_gain();
    measurement_matrix_prep();
    new_state_estimate();
    new_estimate_error();

    return state_matrix;
}

kalman_filter_constant_1d::kalman_filter_constant_1d(double initial_estimate, double initial_estimate_error)
{
    est = initial_estimate;
    E_est = initial_estimate_error;
}

void kalman_filter_constant_1d::update(const double measurement, const double measurement_error)
{
    if (converged)
    {
        return;
    }
    update_gain(measurement_error);
    update_estimate(measurement);
    update_estimate_error();
}

void kalman_filter_constant_1d::update_gain(const double measurement_error)
{
    KG = E_est / (E_est + measurement_error);
}

void kalman_filter_constant_1d::update_estimate(const double measurement)
{
    est = est + KG * (measurement - est);
    if (KG < 0.0005)
    {
        converged = true;
    }
}

void kalman_filter_constant_1d::update_estimate_error()
{
    E_est = (1 - KG) * E_est;
}

double kalman_filter_constant_1d::get_gain() const
{
    return this->KG;
}

// int main()
// {
//     kalman_filter_constant_1d kf{68, 2};
//     kf.update(75, 4);
//     std::cout << kf.est << std::endl;
//     kf.update(71, 4);
//     std::cout << kf.est << std::endl;
//     kf.update(70, 4);
//     std::cout << kf.est << std::endl;
//     kf.update(74, 4);
//     std::cout << kf.est << std::endl;
//     return 0;
// }

/*
//Example set up of a simple 2-D kalman filter
int main()
{

    //matrices sizes
    MatrixXd X(2, 1);
    MatrixXd P(2, 2);
    MatrixXd U(1, 1);
    MatrixXd A(2, 2);
    MatrixXd C(2, 2);
    MatrixXd m(2, 1); //ignored for the sake of this example
    MatrixXd r(2, 2);
    MatrixXd h(2, 2);
    MatrixXd i(2, 2);
    MatrixXd w(2, 1); //ignored for the sake of this example
    MatrixXd q(2, 2); //ignored for the sake of this example

    //initail values:
    //errors
    w(0, 0) = 0;
    w(1, 0) = 0;

    q(0, 0) = 0;
    q(1, 0) = 0;
    q(0, 1) = 0;
    q(1, 1) = 0;

    m(0, 0) = 0;
    m(1, 0) = 0;

    X(0, 0) = 4000;

    X(1, 0) = 280;

    U(0, 0) = 2;

    A(0, 0) = 1;
    A(1, 0) = 0;
    A(0, 1) = 1;
    A(1, 1) = 1;

    P(0, 0) = 400;
    P(1, 0) = 0;
    P(0, 1) = 0;
    P(1, 1) = 25;

    h(0, 0) = 1;
    h(1, 0) = 0;
    h(0, 1) = 0;
    h(1, 1) = 1;

    r(0, 0) = 625;
    r(1, 0) = 0;
    r(0, 1) = 0;
    r(1, 1) = 36;

    C(0, 0) = 1;
    C(1, 0) = 0;
    C(0, 1) = 0;
    C(1, 1) = 1;

    i(0, 0) = 1;
    i(1, 0) = 0;
    i(0, 1) = 0;
    i(1, 1) = 1;

    kalman_filter filter(X, P, 1, U, A, C, m, r, h, i, w, q);

    //example sensor data
    MatrixXd observations[4];
    MatrixXd a(2, 1);
    MatrixXd b(2, 1);
    MatrixXd c(2, 1);
    MatrixXd d(2, 1);
    observations[0] = a;
    observations[1] = b;
    observations[2] = c;
    observations[3] = d;

    observations[0](0, 0) = 4260;
    observations[0](1, 0) = 282;
    observations[1](0, 0) = 4550;
    observations[1](1, 0) = 285;
    observations[2](0, 0) = 4860;
    observations[2](1, 0) = 286;
    observations[3](0, 0) = 5110;
    observations[3](1, 0) = 290;

    for (int i = 0; i <= 3; i++)
    {
        std::cout << "i: " << i << std::endl;
        X = filter.kalman_one_cycle(observations[i]);
        std::cout << "State_matrix: " << X << "\t" << "\n" << std::endl;
    }
    return -1;
}
*/