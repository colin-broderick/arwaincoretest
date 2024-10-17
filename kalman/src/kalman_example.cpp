#include <thread>
#include <chrono>
#include <mutex>
#include <deque>

#include "kalman.hpp"
#include "../vector3/vector3.hpp"

extern std::mutex PRESSURE_BUFFER_LOCK;
extern std::deque<Vector3> PRESSURE_BUFFER;
extern std::deque<ImuData> IMU_WORLD_BUFFER;

/** \brief runs a kalman filter to determine the altiude*/ //errors to be determined
void kalman()
{
    std::this_thread::sleep_for(2000_ms);

    double altitude_offset;
    {
        std::lock_guard<std::mutex> lock{PRESSURE_BUFFER_LOCK};
        altitude_offset = PRESSURE_BUFFER.back().z;
    }
    double alt = 0;
    double alt_prev = 0;
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
    {
        std::lock_guard<std::mutex> lock{PRESSURE_BUFFER_LOCK};
        U(0, 0) = IMU_WORLD_BUFFER.back().acce.z - 9.81;
    }
    
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

    double dt = 0.1;

    KalmanFilter filter(state_matrix, P, 0.1, U, A, blank_2d, m, r, blank_2d, blank_2d, w, q); //parameters to determine

    while (!SHUTDOWN)
    {

        //get sensor reading
        {
            std::lock_guard<std::mutex> lock{PRESSURE_BUFFER_LOCK};
            alt = PRESSURE_BUFFER.back().z - altitude_offset;
        }

        observation(0,0) = alt;
        observation(1,0) = (alt - alt_prev)/dt;
        {
            std::lock_guard<std::mutex> lock{PRESSURE_BUFFER_LOCK};
            U(0, 0) = IMU_WORLD_BUFFER.back().acce.z - 9.81;
        }
        std::cout << U << std::endl;

        std::cout << "State_matrix: " << state_matrix << "\t";
        state_matrix = filter.kalman_one_cycle(observation, U);
        std::cout << "State_matrix: " << state_matrix << "\t"
                  << "\n"
                  << std::endl;


        alt_prev = alt;


        std::this_thread::sleep_for(100_ms);
    }
}
