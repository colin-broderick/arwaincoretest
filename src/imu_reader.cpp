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
    IMU_IIM42652 imu2{arwain::config.imu2_address, arwain::config.imu2_bus};
    IMU_IIM42652 imu3{arwain::config.imu3_address, arwain::config.imu3_bus};
    LIS3MDL magn{arwain::config.magn_address, arwain::config.magn_bus};
    magn.set_calibration(arwain::config.mag_bias, arwain::config.mag_scale);

    // Choose an orientation filter depending on configuration, with Madgwick as default.
    arwain::Madgwick madgwick_filter{1000.0/arwain::Intervals::IMU_READING_INTERVAL, arwain::config.madgwick_beta};
    arwain::Madgwick madgwick_filter_mag{1000.0/arwain::Intervals::IMU_READING_INTERVAL, arwain::config.madgwick_beta_conv};

    // Local buffers for IMU data.
    Vector3 accel_data1;
    Vector3 gyro_data1;
    Vector3 world_accel_data1;
    Vector3 world_gyro_data1;
    Vector3 magnet;
    Quaternion madgwick_quaternion_data1;
    Quaternion madgwick_quaternion_mag_data1;
    euler_orientation_t madgwick_euler_data1;
    euler_orientation_t madgwick_euler_mag_data1;

    // File handles for logging.
    arwain::Logger ori_diff_file;
    arwain::Logger acce_file;
    arwain::Logger world_acce_file;
    arwain::Logger gyro_file;
    arwain::Logger world_gyro_file;
    arwain::Logger madgwick_euler_file;
    arwain::Logger madgwick_euler_mag_file;
    arwain::Logger madgwick_quat_file;

    if (arwain::config.log_to_file)
    {
        // Open file handles for data logging.
        ori_diff_file.open(arwain::folder_date_string + "/ori_diff.txt");
        acce_file.open(arwain::folder_date_string + "/acce.txt");
        world_acce_file.open(arwain::folder_date_string + "/world_acce.txt");
        gyro_file.open(arwain::folder_date_string + "/gyro.txt");
        world_gyro_file.open(arwain::folder_date_string + "/world_gyro.txt");
        madgwick_euler_file.open(arwain::folder_date_string + "/madgwick_euler_orientation.txt");
        madgwick_quat_file.open(arwain::folder_date_string + "/madgwick_game_rv.txt");
        madgwick_euler_mag_file.open(arwain::folder_date_string + "/madgwick_mag_euler_orientation.txt");

        // File headers
        ori_diff_file << "time yaw" << "\n";
        acce_file << "time x y z" << "\n";
        world_acce_file << "time x y z" << "\n";
        gyro_file << "time x y z" << "\n";
        world_gyro_file << "time x y z" << "\n";
        madgwick_euler_file << "time roll pitch yaw" << "\n";
        madgwick_quat_file << "time w x y z" << "\n";
        madgwick_euler_mag_file << "time roll pitch yaw" << "\n";
    }

    // quaternion quat1;//, quat2, quat3, quat_aggregate, magnetovector;
    long cycle_count = 0;

    // Set up timing.
    auto loopTime = std::chrono::system_clock::now(); // Controls the timing of loop iteration.
    auto timeCount = std::chrono::system_clock::now().time_since_epoch().count(); // Provides an accurate count of milliseconds passed since last loop iteration.
    std::chrono::milliseconds interval{arwain::Intervals::IMU_READING_INTERVAL}; // Interval between loop iterations.

    // After the specificed period has passed, set the filter gain to the standard value.
    std::thread quick_convergence{
        [&madgwick_filter_mag]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds{5000});
            madgwick_filter_mag.set_beta(arwain::config.madgwick_beta);
        }
    };

    while (!arwain::shutdown)
    {
        cycle_count++;
        timeCount = std::chrono::system_clock::now().time_since_epoch().count();

        imu1.read_IMU();
        imu2.read_IMU();
        imu3.read_IMU();

        magnet = magn.read();
        magnet.x = magnet.x + magnet.y*arwain::config.mag_scale_xy + magnet.z*arwain::config.mag_scale_xz; // scale/axis correction
        magnet.y = magnet.y + magnet.z*arwain::config.mag_scale_yz; // scale/axis correction
        magnet = {magnet.y, magnet.x, magnet.z}; // align magnetometer with IMU.

        accel_data1 = {imu1.accelerometer_x, imu1.accelerometer_y, imu1.accelerometer_z};
        gyro_data1 = {imu1.gyroscope_x, imu1.gyroscope_y, imu1.gyroscope_z};

        // Adjust for biases according to configuration file.
        accel_data1 = accel_data1 - arwain::config.accel1_bias;
        gyro_data1 = gyro_data1 - arwain::config.gyro1_bias;

        { // Add new reading to end of buffer, and remove oldest reading from start of buffer.
            std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
            arwain::Buffers::IMU_BUFFER.pop_front();
            arwain::Buffers::IMU_BUFFER.push_back({accel_data1, gyro_data1});
        }

        madgwick_filter.update(
            timeCount,
            gyro_data1.x, gyro_data1.y, gyro_data1.z,
            accel_data1.x, accel_data1.y, accel_data1.z
        );
        madgwick_filter_mag.update(
            timeCount,
            gyro_data1.x, gyro_data1.y, gyro_data1.z,
            accel_data1.x, accel_data1.y, accel_data1.z,
            magnet.x, magnet.y, magnet.z
        );

        // SLERP all orientation filters into a canonical filter.
        // quat1 = {filter1->getW(), filter1->getX(), filter1->getY(), filter1->getZ()};
        // quat2 = {filter2->getW(), filter2->getX(), filter2->getY(), filter2->getZ()};
        // quat3 = {filter3->getW(), filter3->getX(), filter3->getY(), filter3->getZ()};
        // quat_aggregate = quaternion::nslerp(quaternion::nslerp(quat1, quat2, 0.5), quat3, 0.6666666);

        // Compute the SLERP factor based on how 'wrong' the magnetic orientation is compared to the gyro orientation.
        // double s = gyro_mag_co_trust(quat_aggregate, magnetovector);
        // SLERP the two orientation vectors using the computed trust factor.
        // quat_aggregate = quaternion::nslerp(quat_aggregate, magnetovector, 1 - s);

        // Back SLERP
        // quat1 = quaternion::slerp(quat1, quat_aggregate, 0.02);
        // quat2 = quaternion::slerp(quat2, quat_aggregate, 0.02);
        // quat3 = quaternion::slerp(quat3, quat_aggregate, 0.02);
        // filter1->setQ(quat1.w, quat1.x, quat1.y, quat1.z);
        // filter2->setQ(quat2.w, quat2.x, quat2.y, quat2.z);
        // filter3->setQ(quat3.w, quat3.x, quat3.y, quat3.z);

        // Extract Euler orientation from filter.

        madgwick_quaternion_data1 = {madgwick_filter.getW(), madgwick_filter.getX(), madgwick_filter.getY(), madgwick_filter.getZ()};
        madgwick_quaternion_mag_data1 = {madgwick_filter_mag.getW(), madgwick_filter_mag.getX(), madgwick_filter_mag.getY(), madgwick_filter_mag.getZ()};
        madgwick_euler_data1 = compute_euler(madgwick_quaternion_data1);
        madgwick_euler_mag_data1 = compute_euler(madgwick_quaternion_mag_data1);

        // auto quat_diff = madgwick_quaternion_data1 * madgwick_quaternion_mag_data1.conjugate();
        // arwain::yaw_offset = compute_euler(quat_diff).yaw;

        { // Add orientation information to buffers.
            std::lock_guard<std::mutex> lock{arwain::Locks::ORIENTATION_BUFFER_LOCK};
            arwain::Buffers::EULER_ORIENTATION_BUFFER.pop_front();
            arwain::Buffers::EULER_ORIENTATION_BUFFER.push_back(madgwick_euler_data1);
            arwain::Buffers::QUAT_ORIENTATION_BUFFER.pop_front();
            arwain::Buffers::QUAT_ORIENTATION_BUFFER.push_back(madgwick_quaternion_data1);
        }

        // Add world-aligned IMU to its own buffer.
        world_accel_data1 = world_align(accel_data1, madgwick_quaternion_data1);
        world_gyro_data1 = world_align(gyro_data1, madgwick_quaternion_data1);
        {
            std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
            arwain::Buffers::IMU_WORLD_BUFFER.pop_front();
            arwain::Buffers::IMU_WORLD_BUFFER.push_back({world_accel_data1, world_gyro_data1});
        }

        // Compute the new correction based on magn/gyro filter diffs.
        double new_yaw_offset = unwrap_phase_degrees(madgwick_filter_mag.getYawRadians() - madgwick_filter.getYawRadians(), arwain::yaw_offset);
        if (arwain::yaw_offset == 0)
        {
            arwain::yaw_offset = new_yaw_offset;
        }
        else
        {
            arwain::yaw_offset = new_yaw_offset*0.001 + 0.999*arwain::yaw_offset;
        }

        // Write all log files.
        if (arwain::config.log_to_file)
        {
            ori_diff_file << timeCount << " " << arwain::yaw_offset << "\n";
            acce_file << timeCount << " " << accel_data1.x << " " << accel_data1.y << " " << accel_data1.z << "\n";
            gyro_file << timeCount << " " << gyro_data1.x << " " << gyro_data1.y << " " << gyro_data1.z << "\n";
            world_acce_file << timeCount << " " << world_accel_data1.x << " " << world_accel_data1.y << " " << world_accel_data1.z << "\n";
            world_gyro_file << timeCount << " " << world_gyro_data1.x << " " << world_gyro_data1.y << " " << world_gyro_data1.z << "\n";
            madgwick_euler_file << timeCount << " " << madgwick_euler_data1.roll << " " << madgwick_euler_data1.pitch << " " << madgwick_euler_data1.yaw << "\n";
            madgwick_euler_mag_file << timeCount << " " << madgwick_euler_mag_data1.roll << " " << madgwick_euler_mag_data1.pitch << " " << madgwick_euler_mag_data1.yaw << "\n";
            madgwick_quat_file << timeCount << " " << madgwick_quaternion_data1.w << " " << madgwick_quaternion_data1.x << " " << madgwick_quaternion_data1.y << " " << madgwick_quaternion_data1.z << "\n";
        }

        // Wait until the next tick.
        loopTime = loopTime + interval;
        std::this_thread::sleep_until(loopTime);
    }

    quick_convergence.join();

    // Close all file handles.
    if (arwain::config.log_to_file)
    {
        ori_diff_file.close();
        acce_file.close();
        world_acce_file.close();
        gyro_file.close();
        world_gyro_file.close();
        madgwick_euler_file.close();
        madgwick_quat_file.close();
        madgwick_euler_mag_file.close();
    }
}
