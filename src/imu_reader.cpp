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

/** \brief Rotates a 3-vector according to a quaternion orientation.
 * \param vec The 3-vector to rotate.
 * \param orientation The rotation to apply to the 3-vector.
 */
static vector3 world_align(const vector3& vec, const quaternion& rotation)
{
    // Convert the 3-vector into a quaternion.
    quaternion quat_vec{0, vec.x, vec.y, vec.z};
    quaternion quat_ori{rotation.w, rotation.x, rotation.y, rotation.z};

    // Compute the rotated vector as a quaternion.
    quaternion rotated_quaternion_vector = quat_ori * quat_vec * quat_ori.conjugate();

    // Cast the rotated quaternion back into a 3-vector.
    return vector3{
        (double)rotated_quaternion_vector.x,
        (double)rotated_quaternion_vector.y,
        (double)rotated_quaternion_vector.z
    };
}

static euler_orientation_t compute_euler(quaternion& q)
{
    euler_orientation_t euler;
    euler.roll = std::atan2(q.w*q.x + q.y*q.z, 0.5f - q.x*q.x - q.y*q.y);
	euler.pitch = std::asin(-2.0 * (q.x*q.z - q.w*q.y));
	euler.yaw = std::atan2(q.x*q.y + q.w*q.z, 0.5 - q.y*q.y - q.z*q.z);
    return euler;
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

    IMU_IIM42652 imu1{arwain::config.imu1_address, arwain::config.imu1_bus};
    IMU_IIM42652 imu2{arwain::config.imu2_address, arwain::config.imu2_bus};
    IMU_IIM42652 imu3{arwain::config.imu3_address, arwain::config.imu3_bus};

    // Choose an orientation filter depending on configuration, with Madgwick as default.
    arwain::Filter* filter1;
    arwain::Filter* filter2;
    arwain::Filter* filter3;
    if (arwain::config.orientation_filter == "efaroe")
    {
        filter1 = new arwain::eFaroe{quaternion{0,0,0,0}, arwain::config.gyro1_bias, 100, arwain::config.efaroe_beta, arwain::config.efaroe_zeta};
        filter2 = new arwain::eFaroe{quaternion{0,0,0,0}, arwain::config.gyro2_bias, 100, arwain::config.efaroe_beta, arwain::config.efaroe_zeta};
        filter3 = new arwain::eFaroe{quaternion{0,0,0,0}, arwain::config.gyro3_bias, 100, arwain::config.efaroe_beta, arwain::config.efaroe_zeta};
    }
    else if (arwain::config.orientation_filter == "madgwick")
    {
        filter1 = new arwain::Madgwick{1000.0/arwain::Intervals::IMU_READING_INTERVAL, arwain::config.madgwick_beta};
        filter2 = new arwain::Madgwick{1000.0/arwain::Intervals::IMU_READING_INTERVAL, arwain::config.madgwick_beta};
        filter3 = new arwain::Madgwick{1000.0/arwain::Intervals::IMU_READING_INTERVAL, arwain::config.madgwick_beta};
    }

    // Local buffers for IMU data.
    vector3 accel_data1;
    vector3 accel_data2;
    vector3 accel_data3;
    vector3 gyro_data1;
    vector3 gyro_data2;
    vector3 gyro_data3;
    vector3 world_accel_data1;
    vector3 world_gyro_data1;
    euler_orientation_t euler_data;
    quaternion quat_data;

    // File handles for logging.
    arwain::Logger acce_file;
    arwain::Logger gyro_file;
    arwain::Logger euler_file;
    arwain::Logger quat_file;
    arwain::Logger multi_quat_file;

    if (arwain::config.log_to_file)
    {
        // Open file handles for data logging.
        acce_file.open(arwain::folder_date_string + "/acce.txt");
        gyro_file.open(arwain::folder_date_string + "/gyro.txt");
        euler_file.open(arwain::folder_date_string + "/euler_orientation.txt");
        quat_file.open(arwain::folder_date_string + "/game_rv.txt");
        multi_quat_file.open(arwain::folder_date_string + "/multi_quat.txt");

        // File headers
        acce_file << "# time x y z" << "\n";
        gyro_file << "# time x y z" << "\n";
        euler_file << "# time roll pitch yaw" << "\n";
        quat_file << "# time w x y z" << "\n";
        multi_quat_file << "# time q1w q1x q1y q1z q2w q2x q2y q2z q3w q3x q3y q3z" << "\n";
    }

    quaternion quat1, quat2, quat3;
    int cycle_count = 0;

    // Set up timing.
    auto loopTime = std::chrono::system_clock::now(); // Controls the timing of loop iteration.
    auto timeCount = std::chrono::system_clock::now().time_since_epoch().count(); // Provides an accurate count of milliseconds passed since last loop iteration.
    std::chrono::milliseconds interval{arwain::Intervals::IMU_READING_INTERVAL}; // Interval between loop iterations.


    while (!arwain::shutdown)
    {
        cycle_count++;
        timeCount = std::chrono::system_clock::now().time_since_epoch().count();

        imu1.read_IMU();
        imu2.read_IMU();
        imu3.read_IMU();

        accel_data1 = {imu1.accelerometer_x, imu1.accelerometer_y, imu1.accelerometer_z};
        gyro_data1 = {imu1.gyroscope_x, imu1.gyroscope_y, imu1.gyroscope_z};
        accel_data2 = {imu2.accelerometer_x, imu2.accelerometer_y, imu2.accelerometer_z};
        gyro_data2 = {imu2.gyroscope_x, imu2.gyroscope_y, imu2.gyroscope_z};
        accel_data3 = {imu3.accelerometer_x, imu3.accelerometer_y, imu3.accelerometer_z};
        gyro_data3 = {imu3.gyroscope_x, imu3.gyroscope_y, imu3.gyroscope_z};

        // Adjust for biases according to configuration file.
        accel_data1 = accel_data1 - arwain::config.accel1_bias;
        gyro_data1 = gyro_data1 - arwain::config.gyro1_bias;
        accel_data2 = accel_data2 - arwain::config.accel2_bias;
        gyro_data2 = gyro_data2 - arwain::config.gyro2_bias;
        accel_data3 = accel_data3 - arwain::config.accel3_bias;
        gyro_data3 = gyro_data3 - arwain::config.gyro3_bias;

        { // Add new reading to end of buffer, and remove oldest reading from start of buffer.
            std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
            arwain::Buffers::IMU_BUFFER.pop_front();
            arwain::Buffers::IMU_BUFFER.push_back({accel_data1, gyro_data1});
        }

        // Log IMU to file.
        if (arwain::config.log_to_file)
        {
            acce_file << timeCount << " " << accel_data1.x << " " << accel_data1.y << " " << accel_data1.z << "\n";
            gyro_file << timeCount << " " << gyro_data1.x << " " << gyro_data1.y << " " << gyro_data1.z << "\n";
        }

        filter1->update(
            timeCount,
            gyro_data1.x, gyro_data1.y, gyro_data1.z,
            accel_data1.x, accel_data1.y, accel_data1.z
        );
        filter2->update(
            timeCount,
            gyro_data2.x, gyro_data2.y, gyro_data2.z,
            accel_data2.x, accel_data2.y, accel_data2.z
        );
        filter3->update(
            timeCount,
            gyro_data3.x, gyro_data3.y, gyro_data3.z,
            accel_data3.x, accel_data3.y, accel_data3.z
        );

        // SLERP all orientation filters into a canonical filter.
        quat1 = {filter1->getW(), filter1->getX(), filter1->getY(), filter1->getZ()};
        quat2 = {filter2->getW(), filter2->getX(), filter2->getY(), filter2->getZ()};
        quat3 = {filter3->getW(), filter3->getX(), filter3->getY(), filter3->getZ()};
        quat_data = quaternion::nslerp(quaternion::nslerp(quat1, quat2, 0.5), quat3, 0.6666666);

        // Back SLERP
        quat1 = quaternion::slerp(quat1, quat_data, 0.02);
        quat2 = quaternion::slerp(quat2, quat_data, 0.02);
        quat3 = quaternion::slerp(quat3, quat_data, 0.02);
        filter1->setQ(quat1.w, quat1.x, quat1.y, quat1.z);
        filter2->setQ(quat2.w, quat2.x, quat2.y, quat2.z);
        filter3->setQ(quat3.w, quat3.x, quat3.y, quat3.z);

        if (cycle_count % 100 == 0)
        {
            multi_quat_file << timeCount << " "
                            << filter1->getW() << " " << filter1->getX() << " " << filter1->getY() << " " << filter1->getZ() << " "
                            << filter2->getW() << " " << filter2->getX() << " " << filter2->getY() << " " << filter2->getZ() << " "
                            << filter3->getW() << " " << filter3->getX() << " " << filter3->getY() << " " << filter3->getZ() << "\n";
        }

        // Extract Euler orientation from filter.
        euler_data = compute_euler(quat_data);

        { // Add orientation information to buffers.
            std::lock_guard<std::mutex> lock{arwain::Locks::ORIENTATION_BUFFER_LOCK};
            arwain::Buffers::EULER_ORIENTATION_BUFFER.pop_front();
            arwain::Buffers::EULER_ORIENTATION_BUFFER.push_back(euler_data);
            arwain::Buffers::QUAT_ORIENTATION_BUFFER.pop_front();
            arwain::Buffers::QUAT_ORIENTATION_BUFFER.push_back(quat_data);
        }

        // Add world-aligned IMU to its own buffer.
        world_accel_data1 = world_align(accel_data1, quat_data);
        world_gyro_data1 = world_align(gyro_data1, quat_data);
        {
            std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
            arwain::Buffers::IMU_WORLD_BUFFER.pop_front();
            arwain::Buffers::IMU_WORLD_BUFFER.push_back({world_accel_data1, world_gyro_data1});
        }

        // Log orientation information to file.
        if (arwain::config.log_to_file)
        {
            euler_file << timeCount << " " << euler_data.roll << " " << euler_data.pitch << " " << euler_data.yaw << "\n";
            quat_file << timeCount << " " << quat_data.w << " " << quat_data.x << " " << quat_data.y << " " << quat_data.z << "\n";
        }

        // Wait until the next tick.
        loopTime = loopTime + interval;
        std::this_thread::sleep_until(loopTime);
    }

    delete filter1;
    delete filter2;
    delete filter3;

    // Close all file handles.
    if (arwain::config.log_to_file)
    {
        acce_file.close();
        gyro_file.close();
        euler_file.close();
        quat_file.close();
        multi_quat_file.close();
    }
}
