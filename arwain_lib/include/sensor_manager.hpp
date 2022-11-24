#ifndef _ARWAIN_IMU_READER_HPP
#define _ARWAIN_IMU_READER_HPP

#include <functional>

#include "arwain.hpp"
#include "logger.hpp"
#include "arwain_thread.hpp"
#include "iim42652.hpp"
#include "lis3mdl.hpp"
#include "madgwick.hpp"
#include "arwain_utils.hpp"

class SensorManager
{
    TESTABLE:
        void run();
        void run_inference();
        void setup_inference();
        arwain::ReturnCode cleanup_inference();
        void run_gyro_calibration();
        void run_magn_calibration();
        void core_setup();
        void run_accel_calibration();
        void run_idle();
        void run_test_stance_detector();
        void run_self_test();
        Vector3 world_align(const Vector3& vec, const Quaternion& rotation);

    private:
        // Callback to be executed when active gyroscope calibration is complete.
        std::function<void()> post_gyro_calib_callback;

        ArwainThread job_thread;
        ArwainThread quick_madgwick_convergence_thread;
        IMU_IIM42652 imu1;
        IMU_IIM42652 imu2;
        IMU_IIM42652 imu3;
        LIS3MDL magnetometer;
        arwain::Madgwick madgwick_filter_1;
        arwain::Madgwick madgwick_filter_mag_1;

        // File handles for logging.
        arwain::Logger ori_diff_file;
        arwain::Logger acce_file_1;
        arwain::Logger world_acce_file_1;
        arwain::Logger gyro_file_1;
        arwain::Logger world_gyro_file_1;
        arwain::Logger madgwick_euler_file_1;
        arwain::Logger madgwick_euler_mag_file_1;
        arwain::Logger madgwick_quat_file_1;
        arwain::Logger imu_calib_file_1;
        arwain::Logger imu_calib_file_2;
        arwain::Logger imu_calib_file_3;

        RollingAverage rolling_average_accel_z_for_altimeter;

    public:
        SensorManager();
        bool init();
        void join();
        void set_post_gyro_calibration_callback(std::function<void()> func);
};

#endif
