//includes
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
//defines

//namespace
using Eigen::MatrixXd;

//kalman filter class
class KalmanFilter
{
public:
    KalmanFilter() = default;
    KalmanFilter(MatrixXd X, MatrixXd P, int dt, MatrixXd u, MatrixXd a, MatrixXd c, MatrixXd m, MatrixXd r, MatrixXd h, MatrixXd i, MatrixXd w, MatrixXd q)
    {
        state_matrix = X;
        state_covariance_matrix = P;
        control_variable_matrix = u;
        set_B(dt);
        A = a;
        C = c;
        measurement_noice = m;
        measurement_covariance_matrix = r;
        H = h;
        I = i;
        predicted_state_noise_matrix = w;
        process_noise_covariancce_matrix = q;
    }
    MatrixXd kalman_one_cycle(MatrixXd observation, MatrixXd U);
    
private:
    void set_B(double dt);
    void new_predicted_state();
    void new_state_covariance_matrix();
    void kalman_gain();
    void new_state_estimate();
    void new_estimate_error();
    void measurement_matrix_prep();

    MatrixXd state_matrix;            // initial state/estimates
    MatrixXd KG;                      //Kalman gain - make private later
    MatrixXd state_covariance_matrix; //error in the estimate/process
    MatrixXd B{2, 1};
    MatrixXd A;
    MatrixXd C;
    MatrixXd Y;
    MatrixXd H;
    MatrixXd I;
    MatrixXd measurement_noice;
    MatrixXd measurement_covariance_matrix;  //R
    MatrixXd sensor_noise_covariance_matrix; //measurement error
    MatrixXd control_variable_matrix;
    MatrixXd predicted_state_noise_matrix;
    MatrixXd process_noise_covariancce_matrix;
};

class KalmanFilter1D
{
    public:
        double est = 0;
        bool converged = false;

    private:
        double KG = 0;
        double E_est = 0;
        double E_mea = 0;

    public:
        KalmanFilter1D() = default;
        KalmanFilter1D(double initial_estimate, double initial_estimate_error);
        void update(const double measurement, const double measurement_error);
        double get_gain() const;

    private:
        void update_gain(const double measurement_error);
        void update_estimate(const double measurement);
        void update_estimate_error();
};
