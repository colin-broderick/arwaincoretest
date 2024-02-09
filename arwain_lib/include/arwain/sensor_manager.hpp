#ifndef _ARWAIN_IMU_READER_HPP
#define _ARWAIN_IMU_READER_HPP

#include <functional>

#include <arwain/devices/iim42652.hpp>
#include <arwain/devices/lis3mdl.hpp>
#include <arwain/orientation/madgwick.hpp>
#include <arwain/logger.hpp>

#include "arwain/i2c_interface.hpp"
#include "arwain/utils.hpp"
#include "arwain/job_interface.hpp"


class SensorManager : public ArwainJob, protected IArwainJobSpec
{
    private:
        void run() override;
        void run_inference() override;
        void setup_inference() override;
        bool cleanup_inference() override;
        void core_setup() override;
        void run_idle() override;
        void run_gyro_calibration();
        void run_magn_calibration();
        void run_accel_calibration();
        void run_test_stance_detector();
        void run_self_test();
        Vector3 world_align(const Vector3& vec, const Quaternion& rotation);

    private:
        // Callback to be executed when active gyroscope calibration is complete.
        std::function<void()> post_gyro_calib_callback;

        std::jthread job_thread;
        std::jthread quick_madgwick_convergence_thread;
        IIM42652<LinuxSmbusI2CDevice> imu1;
        IIM42652<LinuxSmbusI2CDevice> imu2;
        IIM42652<LinuxSmbusI2CDevice> imu3;
        LIS3MDL<LinuxSmbusI2CDevice> magnetometer;
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
        ~SensorManager();
        bool join() override;
        void set_post_gyro_calibration_callback(std::function<void()> func);
        static inline std::string service_name = "SensorManager";
};

#endif
