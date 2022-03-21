// TODO Error checking when starting and using IMUs.

#include <mutex>
#include <deque>
#include <string>
#include <cmath>
#include <thread>

#include "imu_reader.hpp"
#include "multi_imu.hpp"
#include "madgwick.hpp"
#include "efaroe.hpp"
#include "logger.hpp"
#include "arwain.hpp"
#include "timers.hpp"
#include "lis3mdl.hpp"
#include "calibration.hpp"

/** \brief Rotates a 3-vector according to a quaternion orientation.
 * \param vec The 3-vector to rotate.
 * \param orientation The rotation to apply to the 3-vector.
 */
static Vector3 world_align(const Vector3& vec, const Quaternion& rotation)
{
    // Convert the 3-vector into a quaternion.
    Quaternion quat_vec{0, vec.x, vec.y, vec.z};
    Quaternion quat_ori{rotation.w, rotation.x, rotation.y, rotation.z};

    // Compute the rotated vector as a quaternion.
    Quaternion rotated_quaternion_vector = quat_ori * quat_vec * quat_ori.conjugate();

    // Cast the rotated quaternion back into a 3-vector.
    return Vector3{
        (double)rotated_quaternion_vector.x,
        (double)rotated_quaternion_vector.y,
        (double)rotated_quaternion_vector.z
    };
}

static euler_orientation_t compute_euler(Quaternion& q)
{
    euler_orientation_t euler;
    euler.roll = std::atan2(q.w*q.x + q.y*q.z, 0.5 - q.x*q.x - q.y*q.y);
	euler.pitch = std::asin(-2.0 * (q.x*q.z - q.w*q.y));
	euler.yaw = std::atan2(q.x*q.y + q.w*q.z, 0.5 - q.y*q.y - q.z*q.z);
    return euler;
}

/** \brief Compute the relative trust between two sources of orientation, depending on how different
 * they are.
 * 
 * The maximum weighting of the magnetometer is 1%. This drops to approximately 0.2% at a 1 degree
 * difference. The trust in the magnetometer is effectively zero at a 5 degree difference.
 * 
 * \param gyro_orientation The orientation as computed by gyroscope integration.
 * \param mag_orientation The orientation as directly measured by magnetometer.
 * \return How much the magnetometer should be trust.
 */
static double gyro_mag_co_trust(Quaternion gyro_orientation, Quaternion mag_orientation)
{
    // Note; as written, this formula assumes degrees.

    // Determine angle between current gyro orientation and magnetic orientation, dtheta.
    double dtheta = std::abs(Quaternion::angle_between(gyro_orientation, mag_orientation)) * 180.0 / 3.14159;

    // Compute relative trust of the gyroscope.
    return 1.0 / ((dtheta + 1) * (dtheta + 1) * (dtheta + 1)) / 100.0;
}

/** \brief Reads IMU data, runs orientation filter(s), rotates IMU data, and buffers everything.
 * This is the root of the entire tree. Quite a lot goes on here and a lot of buffers
 * are fed. This is also the most time-sensitive thread. A single loop must not take
 * longer than the reading interval of the IMU, so be careful when implementing
 * any additional functionality here.
 */
void imu_reader()
{
    // Quit immediately if IMU not enabled.
    if (arwain::config.no_imu)
    {
        return;
    }

    // Prepare sensors.
    IMU_IIM42652 imu1{arwain::config.imu1_address, arwain::config.imu1_bus};
    imu1.set_gyro_bias(arwain::config.gyro1_bias.x, arwain::config.gyro1_bias.y, arwain::config.gyro1_bias.z);
    imu1.set_accel_bias(arwain::config.accel1_bias.x, arwain::config.accel1_bias.y, arwain::config.accel1_bias.z);
    imu1.set_accel_scale(arwain::config.accel1_scale.x, arwain::config.accel1_scale.y, arwain::config.accel1_scale.z);
    imu1.enable_auto_calib();
    IMU_IIM42652 imu2{arwain::config.imu2_address, arwain::config.imu2_bus};
    imu2.set_gyro_bias(arwain::config.gyro2_bias.x, arwain::config.gyro2_bias.y, arwain::config.gyro2_bias.z);
    imu2.set_accel_bias(arwain::config.accel2_bias.x, arwain::config.accel2_bias.y, arwain::config.accel2_bias.z);
    imu2.set_accel_scale(arwain::config.accel2_scale.x, arwain::config.accel2_scale.y, arwain::config.accel2_scale.z);
    imu2.enable_auto_calib();
    IMU_IIM42652 imu3{arwain::config.imu3_address, arwain::config.imu3_bus};
    imu3.set_gyro_bias(arwain::config.gyro3_bias.x, arwain::config.gyro3_bias.y, arwain::config.gyro3_bias.z);
    imu3.set_accel_bias(arwain::config.accel3_bias.x, arwain::config.accel3_bias.y, arwain::config.accel3_bias.z);
    imu3.set_accel_scale(arwain::config.accel3_scale.x, arwain::config.accel3_scale.y, arwain::config.accel3_scale.z);
    imu3.enable_auto_calib();
    
    LIS3MDL magnetometer{arwain::config.magn_address, arwain::config.magn_bus};
    magnetometer.set_calibration(
        arwain::config.mag_bias,
        arwain::config.mag_scale,
        {arwain::config.mag_scale_xy, arwain::config.mag_scale_yz, arwain::config.mag_scale_xz}
    );

    // Choose an orientation filter depending on configuration, with Madgwick as default.
    arwain::Madgwick madgwick_filter_1{1000.0/arwain::Intervals::IMU_READING_INTERVAL, 0.1};
    arwain::Madgwick madgwick_filter_2{1000.0/arwain::Intervals::IMU_READING_INTERVAL, 0.1};
    arwain::Madgwick madgwick_filter_3{1000.0/arwain::Intervals::IMU_READING_INTERVAL, 0.1};
    arwain::Madgwick madgwick_filter_mag_1{1000.0/arwain::Intervals::IMU_READING_INTERVAL, arwain::config.madgwick_beta_conv};
    arwain::Madgwick madgwick_filter_mag_2{1000.0/arwain::Intervals::IMU_READING_INTERVAL, arwain::config.madgwick_beta_conv};
    arwain::Madgwick madgwick_filter_mag_3{1000.0/arwain::Intervals::IMU_READING_INTERVAL, arwain::config.madgwick_beta_conv};

    Quaternion test_vector{0, 1, 0, 0};

    // After the specificed period has passed, set the magnetic filter gain to the standard value.
    std::thread quick_convergence{
        [&madgwick_filter_mag_1, &madgwick_filter_mag_2, &madgwick_filter_mag_3]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds{5000});
            madgwick_filter_mag_1.set_beta(arwain::config.madgwick_beta);
            madgwick_filter_mag_2.set_beta(arwain::config.madgwick_beta);
            madgwick_filter_mag_3.set_beta(arwain::config.madgwick_beta);
        }
    };

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::GyroscopeCalibration:
            {
                // Give other threads time to settle out of previous mode
                sleep_ms(1000);

                Vector3 results;

                imu1.disable_auto_calib();
                imu2.disable_auto_calib();
                imu3.disable_auto_calib();
                imu1.set_gyro_bias(0, 0, 0);
                imu2.set_gyro_bias(0, 0, 0);
                imu3.set_gyro_bias(0, 0, 0);
                
                // Do the calibration procedure for each gyro and write result to file and into config struct in memory.
                std::cout << "Calibrating gyro 1" << std::endl;
                results = (imu1.calibrate_gyroscope() + imu1.calibrate_gyroscope() + imu1.calibrate_gyroscope()) / 3.0;
                arwain::config.gyro1_bias = results;
                arwain::config.replace("gyro1_bias_x", results.x);
                arwain::config.replace("gyro1_bias_y", results.y);
                arwain::config.replace("gyro1_bias_z", results.z);
                std::cout << "Calibration of gyro 1 complete" << std::endl;

                std::cout << "Calibrating gyro 2" << std::endl;
                results = (imu2.calibrate_gyroscope() + imu2.calibrate_gyroscope() + imu2.calibrate_gyroscope()) / 3.0;
                arwain::config.gyro2_bias = results;
                arwain::config.replace("gyro2_bias_x", results.x);
                arwain::config.replace("gyro2_bias_y", results.y);
                arwain::config.replace("gyro2_bias_z", results.z);
                std::cout << "Calibration of gyro 2 complete" << std::endl;

                std::cout << "Calibrating gyro 3" << std::endl;
                results = (imu3.calibrate_gyroscope() + imu3.calibrate_gyroscope() + imu3.calibrate_gyroscope()) / 3.0;
                arwain::config.gyro3_bias = results;
                arwain::config.replace("gyro3_bias_x", results.x);
                arwain::config.replace("gyro3_bias_y", results.y);
                arwain::config.replace("gyro3_bias_z", results.z);
                std::cout << "Calibration of gyro 3 complete" << std::endl;

                // Reset calibration parameters and re-enable autocalibration.
                imu1.set_gyro_bias(arwain::config.gyro1_bias.x, arwain::config.gyro1_bias.y, arwain::config.gyro1_bias.z);
                imu2.set_gyro_bias(arwain::config.gyro2_bias.x, arwain::config.gyro2_bias.y, arwain::config.gyro2_bias.z);
                imu3.set_gyro_bias(arwain::config.gyro3_bias.x, arwain::config.gyro3_bias.y, arwain::config.gyro3_bias.z);
                imu1.enable_auto_calib();
                imu2.enable_auto_calib();
                imu3.enable_auto_calib();

                // Set mode back to idle
                arwain::system_mode = arwain::OperatingMode::AutoCalibration;

                break;
            }
            case arwain::OperatingMode::MagnetometerCalibration:
            {
                // Unset current calibration config.
                magnetometer.set_calibration(
                    {0, 0, 0},
                    {1, 1, 1},
                    {0, 0, 0}
                );

                // Perform magnetometer calibration procedure.
                arwain::MagnetometerCalibrator clbr;
                std::cout << "About to start magnetometer calibration" << std::endl;
                std::cout << "Move the device through all orientations" << std::endl;
                sleep_ms(1000);
                std::cout << "Calibration started ..." << std::endl;

                while (clbr.get_sphere_coverage_quality() < 60)
                {
                    clbr.feed(magnetometer.read());
                    sleep_ms(100);
                }
                auto [bias, scale] = clbr.solve();

                std::cout << "Bias: " << bias[0] << " " << bias[1] << " " << bias[2] << "\n";
                std::cout << "Scale:\n";
                std::cout << scale[0][0] << " " << scale[0][1] << " " << scale[0][2] << std::endl;
                std::cout << scale[1][0] << " " << scale[1][1] << " " << scale[1][2] << std::endl;
                std::cout << scale[2][0] << " " << scale[2][1] << " " << scale[2][2] << std::endl;
                std::cout << std::endl;

                // Write results into config in memory, and to file.
                arwain::config.replace("mag_bias_x", bias[0]);
                arwain::config.replace("mag_bias_y", bias[1]);
                arwain::config.replace("mag_bias_z", bias[2]);
                arwain::config.replace("mag_scale_x", scale[0][0]);
                arwain::config.replace("mag_scale_y", scale[1][1]);
                arwain::config.replace("mag_scale_z", scale[2][2]);
                arwain::config.replace("mag_scale_xy", scale[0][1]);
                arwain::config.replace("mag_scale_xz", scale[0][2]);
                arwain::config.replace("mag_scale_yz", scale[1][2]);
                arwain::config.mag_bias = {bias[0], bias[1], bias[2]};
                arwain::config.mag_scale = {scale[0][0], scale[1][1], scale[2][2]};
                arwain::config.mag_scale_xy = scale[0][1];
                arwain::config.mag_scale_xz = scale[0][2];
                arwain::config.mag_scale_yz = scale[1][2];

                // Enable new calibration parameters.
                magnetometer.set_calibration(
                    arwain::config.mag_bias,
                    arwain::config.mag_scale,
                    {arwain::config.mag_scale_xy, arwain::config.mag_scale_yz, arwain::config.mag_scale_xz}
                );

                std::cout << "Magnetometer calibration complete" << std::endl;

                arwain::system_mode = arwain::OperatingMode::AutoCalibration;

                break;
            }
            case arwain::OperatingMode::AccelerometerCalibration:
            {
                // TODO
                break;
            }
            case arwain::OperatingMode::Inference:
            {
                // mode-specific setup should go here, before the while loop. e.g. open new log files. ----------------
                
                // File handles for logging.
                arwain::Logger quat_diff_file;
                arwain::Logger ori_diff_file;
                arwain::Logger acce_file_1;
                arwain::Logger world_acce_file_1;
                arwain::Logger gyro_file_1;
                arwain::Logger world_gyro_file_1;
                arwain::Logger madgwick_euler_file_1;
                arwain::Logger madgwick_euler_file_2;
                arwain::Logger madgwick_euler_file_3;
                arwain::Logger madgwick_euler_mag_file_1;
                arwain::Logger madgwick_euler_mag_file_2;
                arwain::Logger madgwick_euler_mag_file_3;
                arwain::Logger madgwick_quat_file_1;
                arwain::Logger madgwick_quat_file_2;
                arwain::Logger madgwick_quat_file_3;
                arwain::Logger imu_calib_file_1;
                arwain::Logger imu_calib_file_2;
                arwain::Logger imu_calib_file_3;

                // Open file handles for data logging.
                quat_diff_file.open(arwain::folder_date_string + "/quat_delta_angle.txt");
                ori_diff_file.open(arwain::folder_date_string + "/ori_diff.txt");
                acce_file_1.open(arwain::folder_date_string + "/acce.txt");
                world_acce_file_1.open(arwain::folder_date_string + "/world_acce.txt");
                gyro_file_1.open(arwain::folder_date_string + "/gyro.txt");
                world_gyro_file_1.open(arwain::folder_date_string + "/world_gyro.txt");
                madgwick_euler_file_1.open(arwain::folder_date_string + "/madgwick_euler_orientation_1.txt");
                madgwick_euler_file_2.open(arwain::folder_date_string + "/madgwick_euler_orientation_2.txt");
                madgwick_euler_file_3.open(arwain::folder_date_string + "/madgwick_euler_orientation_3.txt");
                madgwick_quat_file_1.open(arwain::folder_date_string + "/madgwick_game_rv_1.txt");
                madgwick_quat_file_2.open(arwain::folder_date_string + "/madgwick_game_rv_2.txt");
                madgwick_quat_file_3.open(arwain::folder_date_string + "/madgwick_game_rv_3.txt");
                madgwick_euler_mag_file_1.open(arwain::folder_date_string + "/madgwick_mag_euler_orientation_1.txt");
                madgwick_euler_mag_file_2.open(arwain::folder_date_string + "/madgwick_mag_euler_orientation_2.txt");
                madgwick_euler_mag_file_3.open(arwain::folder_date_string + "/madgwick_mag_euler_orientation_3.txt");
                imu_calib_file_1.open(arwain::folder_date_string + "/imu_calib_1.txt");
                imu_calib_file_2.open(arwain::folder_date_string + "/imu_calib_2.txt");
                imu_calib_file_3.open(arwain::folder_date_string + "/imu_calib_3.txt");

                // File headers
                quat_diff_file << "time diff_q1_q2 diff_q1_q3" << "\n";
                ori_diff_file << "time yaw" << "\n";
                acce_file_1 << "time x y z" << "\n";
                world_acce_file_1 << "time x y z" << "\n";
                gyro_file_1 << "time x y z" << "\n";
                world_gyro_file_1 << "time x y z" << "\n";
                madgwick_euler_file_1 << "time roll pitch yaw" << "\n";
                madgwick_euler_file_2 << "time roll pitch yaw" << "\n";
                madgwick_euler_file_3 << "time roll pitch yaw" << "\n";
                madgwick_quat_file_1 << "time w x y z" << "\n";
                madgwick_quat_file_2 << "time w x y z" << "\n";
                madgwick_quat_file_3 << "time w x y z" << "\n";
                madgwick_euler_mag_file_1 << "time roll pitch yaw" << "\n";
                madgwick_euler_mag_file_2 << "time roll pitch yaw" << "\n";
                madgwick_euler_mag_file_3 << "time roll pitch yaw" << "\n";
                imu_calib_file_1 << "time gx_bias gy_bias gz_bias" << "\n";
                imu_calib_file_2 << "time gx_bias gy_bias gz_bias" << "\n";
                imu_calib_file_3 << "time gx_bias gy_bias gz_bias" << "\n";

                // Set up timing.
                auto loopTime = std::chrono::system_clock::now(); // Controls the timing of loop iteration.
                auto timeCount = std::chrono::system_clock::now().time_since_epoch().count(); // Provides an accurate count of milliseconds passed since last loop iteration.
                std::chrono::milliseconds interval{arwain::Intervals::IMU_READING_INTERVAL}; // Interval between loop iterations.

                // end of mode-specific setup ------------------------------------------------------------------------

                while (arwain::system_mode == arwain::OperatingMode::Inference)
                {
                    timeCount = std::chrono::system_clock::now().time_since_epoch().count();

                    auto [accel_data1, gyro_data1] = imu1.read_IMU();
                    auto [accel_data2, gyro_data2] = imu2.read_IMU();
                    auto [accel_data3, gyro_data3] = imu3.read_IMU();
                    // Force approximate alignment of the IMUs.
                    accel_data2 = {accel_data2.y, -accel_data2.x, accel_data2.z};
                    gyro_data2 = {gyro_data2.y, -gyro_data2.x, gyro_data2.z};
                    accel_data3 = {accel_data3.y, -accel_data3.x, accel_data3.z};
                    gyro_data3 = {gyro_data3.y, -gyro_data3.x, gyro_data3.z};

                    if (arwain::request_gyro_calib)
                    {
                        std::cout << imu1.get_gyro_calib() << "      ";
                        std::cout << imu2.get_gyro_calib() << "      ";
                        std::cout << imu3.get_gyro_calib() << std::endl;
                        arwain::request_gyro_calib = false;
                    }

                    Vector3 magnet = magnetometer.read();
                    magnet = {magnet.y, magnet.x, magnet.z}; // align magnetometer with IMU.

                    { // Add new reading to end of buffer, and remove oldest reading from start of buffer.
                        std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
                        arwain::Buffers::IMU_BUFFER.pop_front();
                        arwain::Buffers::IMU_BUFFER.push_back({accel_data1, gyro_data1});
                    }

                    madgwick_filter_1.update(timeCount, gyro_data1.x, gyro_data1.y, gyro_data1.z, accel_data1.x, accel_data1.y, accel_data1.z);
                    madgwick_filter_2.update(timeCount, gyro_data2.x, gyro_data2.y, gyro_data2.z, accel_data2.x, accel_data2.y, accel_data2.z);
                    madgwick_filter_3.update(timeCount, gyro_data3.x, gyro_data3.y, gyro_data3.z, accel_data3.x, accel_data3.y, accel_data3.z);
                    madgwick_filter_mag_1.update(timeCount, gyro_data1.x, gyro_data1.y, gyro_data1.z, accel_data1.x, accel_data1.y, accel_data1.z, magnet.x, magnet.y, magnet.z);
                    madgwick_filter_mag_2.update(timeCount, gyro_data2.x, gyro_data2.y, gyro_data2.z, accel_data2.x, accel_data2.y, accel_data2.z, magnet.x, magnet.y, magnet.z);
                    madgwick_filter_mag_3.update(timeCount, gyro_data3.x, gyro_data3.y, gyro_data3.z, accel_data3.x, accel_data3.y, accel_data3.z, magnet.x, magnet.y, magnet.z);

                    // Extract Euler orientation from filters.
                    Quaternion madgwick_quaternion_data1 = {madgwick_filter_1.getW(), madgwick_filter_1.getX(), madgwick_filter_1.getY(), madgwick_filter_1.getZ()};
                    Quaternion madgwick_quaternion_data2 = {madgwick_filter_2.getW(), madgwick_filter_2.getX(), madgwick_filter_2.getY(), madgwick_filter_2.getZ()};
                    Quaternion madgwick_quaternion_data3 = {madgwick_filter_3.getW(), madgwick_filter_3.getX(), madgwick_filter_3.getY(), madgwick_filter_3.getZ()};
                    Quaternion madgwick_quaternion_mag_data1 = {madgwick_filter_mag_1.getW(), madgwick_filter_mag_1.getX(), madgwick_filter_mag_1.getY(), madgwick_filter_mag_1.getZ()};
                    Quaternion madgwick_quaternion_mag_data2 = {madgwick_filter_mag_2.getW(), madgwick_filter_mag_2.getX(), madgwick_filter_mag_2.getY(), madgwick_filter_mag_2.getZ()};
                    Quaternion madgwick_quaternion_mag_data3 = {madgwick_filter_mag_3.getW(), madgwick_filter_mag_3.getX(), madgwick_filter_mag_3.getY(), madgwick_filter_mag_3.getZ()};
                    euler_orientation_t madgwick_euler_data1 = compute_euler(madgwick_quaternion_data1);
                    euler_orientation_t madgwick_euler_data2 = compute_euler(madgwick_quaternion_data2);
                    euler_orientation_t madgwick_euler_data3 = compute_euler(madgwick_quaternion_data3);
                    euler_orientation_t madgwick_euler_mag_data1 = compute_euler(madgwick_quaternion_mag_data1);
                    euler_orientation_t madgwick_euler_mag_data2 = compute_euler(madgwick_quaternion_mag_data2);
                    euler_orientation_t madgwick_euler_mag_data3 = compute_euler(madgwick_quaternion_mag_data3);

                    { // Add orientation information to buffers.
                        std::lock_guard<std::mutex> lock{arwain::Locks::ORIENTATION_BUFFER_LOCK};
                        arwain::Buffers::EULER_ORIENTATION_BUFFER.pop_front();
                        arwain::Buffers::EULER_ORIENTATION_BUFFER.push_back(madgwick_euler_data1);
                        arwain::Buffers::QUAT_ORIENTATION_BUFFER.pop_front();
                        arwain::Buffers::QUAT_ORIENTATION_BUFFER.push_back(madgwick_quaternion_data1);
                    }

                    // Add world-aligned IMU to its own buffer.
                    Vector3 world_accel_data1 = world_align(accel_data1, madgwick_quaternion_data1);
                    Vector3 world_gyro_data1 = world_align(gyro_data1, madgwick_quaternion_data1);
                    {
                        std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
                        arwain::Buffers::IMU_WORLD_BUFFER.pop_front();
                        arwain::Buffers::IMU_WORLD_BUFFER.push_back({world_accel_data1, world_gyro_data1});
                    }
                    arwain::rolling_average_accel_z_for_altimeter.feed(world_accel_data1.z);

                    // Compute the new correction based on magnetometer/gyro filter diffs.
                    double new_yaw_offset = unwrap_phase_radians(madgwick_filter_mag_1.getYawRadians() - madgwick_filter_1.getYawRadians(), arwain::yaw_offset);
                    if (arwain::yaw_offset == 0)
                    {
                        arwain::yaw_offset = new_yaw_offset;
                    }
                    else
                    {
                        arwain::yaw_offset = new_yaw_offset*0.001 + 0.999*arwain::yaw_offset;
                    }

                    // Write all log files.
                    quat_diff_file << timeCount << " " << 1.0 - std::pow(Quaternion::dot(madgwick_quaternion_data1, madgwick_quaternion_data2), 2) << " " << 1.0 - std::pow(Quaternion::dot(madgwick_quaternion_data1, madgwick_quaternion_data3), 2) << "\n";
                    ori_diff_file << timeCount << " " << arwain::yaw_offset << "\n";
                    acce_file_1 << timeCount << " " << accel_data1.x << " " << accel_data1.y << " " << accel_data1.z << "\n";
                    gyro_file_1 << timeCount << " " << gyro_data1.x << " " << gyro_data1.y << " " << gyro_data1.z << "\n";
                    world_acce_file_1 << timeCount << " " << world_accel_data1.x << " " << world_accel_data1.y << " " << world_accel_data1.z << "\n";
                    world_gyro_file_1 << timeCount << " " << world_gyro_data1.x << " " << world_gyro_data1.y << " " << world_gyro_data1.z << "\n";
                    madgwick_euler_file_1 << timeCount << " " << madgwick_euler_data1.roll << " " << madgwick_euler_data1.pitch << " " << madgwick_euler_data1.yaw << "\n";
                    madgwick_euler_file_2 << timeCount << " " << madgwick_euler_data2.roll << " " << madgwick_euler_data2.pitch << " " << madgwick_euler_data2.yaw << "\n";
                    madgwick_euler_file_3 << timeCount << " " << madgwick_euler_data3.roll << " " << madgwick_euler_data3.pitch << " " << madgwick_euler_data3.yaw << "\n";
                    madgwick_euler_mag_file_1 << timeCount << " " << madgwick_euler_mag_data1.roll << " " << madgwick_euler_mag_data1.pitch << " " << madgwick_euler_mag_data1.yaw << "\n";
                    madgwick_euler_mag_file_2 << timeCount << " " << madgwick_euler_mag_data2.roll << " " << madgwick_euler_mag_data2.pitch << " " << madgwick_euler_mag_data2.yaw << "\n";
                    madgwick_euler_mag_file_3 << timeCount << " " << madgwick_euler_mag_data3.roll << " " << madgwick_euler_mag_data3.pitch << " " << madgwick_euler_mag_data3.yaw << "\n";
                    madgwick_quat_file_1 << timeCount << " " << madgwick_quaternion_data1.w << " " << madgwick_quaternion_data1.x << " " << madgwick_quaternion_data1.y << " " << madgwick_quaternion_data1.z << "\n";
                    madgwick_quat_file_2 << timeCount << " " << madgwick_quaternion_data2.w << " " << madgwick_quaternion_data2.x << " " << madgwick_quaternion_data2.y << " " << madgwick_quaternion_data2.z << "\n";
                    madgwick_quat_file_3 << timeCount << " " << madgwick_quaternion_data3.w << " " << madgwick_quaternion_data3.x << " " << madgwick_quaternion_data3.y << " " << madgwick_quaternion_data3.z << "\n";
                    imu_calib_file_1 << timeCount << " " << imu1.get_gyro_calib_x() << " " << imu1.get_gyro_calib_y() << " " << imu1.get_gyro_calib_z() << "\n";
                    imu_calib_file_2 << timeCount << " " << imu2.get_gyro_calib_x() << " " << imu2.get_gyro_calib_y() << " " << imu2.get_gyro_calib_z() << "\n";
                    imu_calib_file_3 << timeCount << " " << imu3.get_gyro_calib_x() << " " << imu3.get_gyro_calib_y() << " " << imu3.get_gyro_calib_z() << "\n";

                    // Wait until the next tick.
                    loopTime = loopTime + interval;
                    std::this_thread::sleep_until(loopTime);
                }
                break;
            }
            case arwain::OperatingMode::AutoCalibration:
            case arwain::OperatingMode::TestStanceDetector:
            {
                // Set up timing.
                auto loopTime = std::chrono::system_clock::now(); // Controls the timing of loop iteration.
                auto timeCount = std::chrono::system_clock::now().time_since_epoch().count(); // Provides an accurate count of milliseconds passed since last loop iteration.
                std::chrono::milliseconds interval{arwain::Intervals::IMU_READING_INTERVAL}; // Interval between loop iterations.

                while (arwain::system_mode == arwain::OperatingMode::AutoCalibration || arwain::system_mode == arwain::OperatingMode::TestStanceDetector)
                {
                    timeCount = std::chrono::system_clock::now().time_since_epoch().count();

                    auto [accel_data1, gyro_data1] = imu1.read_IMU();
                    auto [accel_data2, gyro_data2] = imu2.read_IMU();
                    auto [accel_data3, gyro_data3] = imu3.read_IMU();
                    // Force approximate alignment of the IMUs.
                    accel_data2 = {accel_data2.y, -accel_data2.x, accel_data2.z};
                    gyro_data2 = {gyro_data2.y, -gyro_data2.x, gyro_data2.z};
                    accel_data3 = {accel_data3.y, -accel_data3.x, accel_data3.z};
                    gyro_data3 = {gyro_data3.y, -gyro_data3.x, gyro_data3.z};

                    if (arwain::request_gyro_calib)
                    {
                        std::cout << imu1.get_gyro_calib() << "      ";
                        std::cout << imu2.get_gyro_calib() << "      ";
                        std::cout << imu3.get_gyro_calib() << std::endl;
                        arwain::request_gyro_calib = false;
                    }

                    Vector3 magnet = magnetometer.read();
                    magnet = {magnet.y, magnet.x, magnet.z}; // align magnetometer with IMU.

                    { // Add new reading to end of buffer, and remove oldest reading from start of buffer.
                        std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
                        arwain::Buffers::IMU_BUFFER.pop_front();
                        arwain::Buffers::IMU_BUFFER.push_back({accel_data1, gyro_data1});
                    }

                    madgwick_filter_1.update(timeCount, gyro_data1.x, gyro_data1.y, gyro_data1.z, accel_data1.x, accel_data1.y, accel_data1.z);
                    madgwick_filter_2.update(timeCount, gyro_data2.x, gyro_data2.y, gyro_data2.z, accel_data2.x, accel_data2.y, accel_data2.z);
                    madgwick_filter_3.update(timeCount, gyro_data3.x, gyro_data3.y, gyro_data3.z, accel_data3.x, accel_data3.y, accel_data3.z);
                    madgwick_filter_mag_1.update(timeCount, gyro_data1.x, gyro_data1.y, gyro_data1.z, accel_data1.x, accel_data1.y, accel_data1.z, magnet.x, magnet.y, magnet.z);

                    // Extract Euler orientation from filters.
                    Quaternion madgwick_quaternion_data1 = {madgwick_filter_1.getW(), madgwick_filter_1.getX(), madgwick_filter_1.getY(), madgwick_filter_1.getZ()};
                    Quaternion madgwick_quaternion_data2 = {madgwick_filter_2.getW(), madgwick_filter_2.getX(), madgwick_filter_2.getY(), madgwick_filter_2.getZ()};
                    Quaternion madgwick_quaternion_data3 = {madgwick_filter_3.getW(), madgwick_filter_3.getX(), madgwick_filter_3.getY(), madgwick_filter_3.getZ()};
                    Quaternion madgwick_quaternion_mag_data1 = {madgwick_filter_mag_1.getW(), madgwick_filter_mag_1.getX(), madgwick_filter_mag_1.getY(), madgwick_filter_mag_1.getZ()};
                    euler_orientation_t madgwick_euler_mag_data1 = compute_euler(madgwick_quaternion_mag_data1);

                    { // Add orientation information to buffers.
                        std::lock_guard<std::mutex> lock{arwain::Locks::ORIENTATION_BUFFER_LOCK};
                        arwain::Buffers::QUAT_ORIENTATION_BUFFER.pop_front();
                        arwain::Buffers::QUAT_ORIENTATION_BUFFER.push_back(madgwick_quaternion_data1);
                    }

                    // Add world-aligned IMU to its own buffer.
                    Vector3 world_accel_data1 = world_align(accel_data1, madgwick_quaternion_data1);
                    Vector3 world_gyro_data1 = world_align(gyro_data1, madgwick_quaternion_data1);
                    {
                        std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
                        arwain::Buffers::IMU_WORLD_BUFFER.pop_front();
                        arwain::Buffers::IMU_WORLD_BUFFER.push_back({world_accel_data1, world_gyro_data1});
                    }
                    arwain::rolling_average_accel_z_for_altimeter.feed(world_accel_data1.z);

                    // Compute the new correction based on magnetometer/gyro filter diffs.
                    double new_yaw_offset = unwrap_phase_radians(madgwick_filter_mag_1.getYawRadians() - madgwick_filter_1.getYawRadians(), arwain::yaw_offset);
                    if (arwain::yaw_offset == 0)
                    {
                        arwain::yaw_offset = new_yaw_offset;
                    }
                    else
                    {
                        arwain::yaw_offset = new_yaw_offset*0.001 + 0.999*arwain::yaw_offset;
                    }

                    // Wait until the next tick.
                    loopTime = loopTime + interval;
                    std::this_thread::sleep_until(loopTime);
                }
                break;
            }
            case arwain::OperatingMode::SelfTest:
            {
                while (arwain::system_mode == arwain::OperatingMode::SelfTest)
                {
                    // Relevant work
                    std::cout << "TODO in selftest mode which is not yet defined" << std::endl;
                    sleep_ms(1000);
                }
                break;
            }
            default:
            {
                sleep_ms(10);
                break;
            }
        }
    }

    // Clean up the thread left by the quick convergence timer.
    quick_convergence.join();
}
