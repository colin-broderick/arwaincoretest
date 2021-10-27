// TODO Error checking when starting and using IMUs.

#include <mutex>
#include <deque>
#include <string>

#include "imu_reader.hpp"
#include "utils.hpp"
#include "multi_imu.hpp"
#include "madgwick.hpp"
#include "efaroe.hpp"
#include "logger.hpp"
#include "shared_resource.hpp"

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

#if IMU_FREQ_200
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

    // IMU_IIM42652 imu{0x69, "/dev/i2c-1"};
    // BMI270 imu{0x69, "/dev/i2c-1"};
    Multi_IIM42652 imu;

    // Choose an orientation filter depending on configuration, with Madgwick as default.
    arwain::Filter* filter;
    if (arwain::config.orientation_filter == "efaroe")
    {
        filter = new arwain::eFaroe{quaternion{0,0,0,0}, arwain::config.gyro_bias, 100, arwain::config.efaroe_beta, arwain::config.efaroe_zeta};
    }
    else
    {
        filter = new arwain::Madgwick{1000.0/arwain::Intervals::IMU_READING_INTERVAL, arwain::config.madgwick_beta};
    }

    int count = 0;
    int get_mag = 0;

    // Local buffers for IMU data.
    vector3 accel_data;
    vector3 gyro_data;
    vector3 mag_data;
    vector3 world_accel_data;
    vector3 world_gyro_data;
    vector3 world_mag_data;
    euler_orientation_t euler_data;
    quaternion quat_data;

    // File handles for logging.
    arwain::Logger acce_file;
    arwain::Logger gyro_file;
    arwain::Logger mag_file;
    arwain::Logger euler_file;
    arwain::Logger quat_file;
    arwain::Logger temp_file;

    if (arwain::config.log_to_file)
    {
        // Open file handles for data logging.
        acce_file.open(arwain::folder_date_string + "/acce.txt");
        gyro_file.open(arwain::folder_date_string + "/gyro.txt");
        mag_file.open(arwain::folder_date_string + "/mag.txt");
        euler_file.open(arwain::folder_date_string + "/euler_orientation.txt");
        quat_file.open(arwain::folder_date_string + "/game_rv.txt");
        temp_file.open(arwain::folder_date_string + "/imu_temperature.txt");

        // File headers
        acce_file << "# time x y z" << "\n";
        gyro_file << "# time x y z" << "\n";
        mag_file << "# time x y z" << "\n";
        euler_file << "# time roll pitch yaw" << "\n";
        quat_file << "# time w x y z" << "\n";
        temp_file << "# time temperature" << "\n";

    }

    // Set up timing.
    auto loopTime = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::now();
    auto timeCount = time.time_since_epoch().count();
    std::chrono::milliseconds interval{arwain::Intervals::IMU_READING_INTERVAL};

    while (!arwain::shutdown)
    {
        time = std::chrono::system_clock::now();
        timeCount = time.time_since_epoch().count();

        if (count % 200 == 0)
        {
            temp_file << timeCount << " " << imu.read_temperature() << "\n";
        }

        // Whether or not to get magnetometer on this spin.
        get_mag = ((count % 20 == 0) && (arwain::config.use_magnetometer || arwain::config.log_magnetometer));

        // Read current sensor values and apply bias correction.
        // imu_error = bmi270_h.get_bmi270_data(&accel_data, &gyro_data);

        imu.read_IMU();
        accel_data = {
            imu.accelerometer_x,
            imu.accelerometer_y,
            imu.accelerometer_z,
        };
        gyro_data = {
            imu.gyroscope_x,
            imu.gyroscope_y,
            imu.gyroscope_z,
        };

        // if (imu_error)
        // { // Log any IMU reading error events.
        //     STATUS.errors = arwain::Errors::IMUReadError;
        //     if (CONFIG.log_to_file)
        //     {
        //         ERROR_LOG << timeCount << " " << "IMU_READ_ERROR" << "\n";
        //     }
        // }

        accel_data = accel_data - arwain::config.accel_bias;
        gyro_data = gyro_data - arwain::config.gyro_bias;

        // Get magnetometer data.
        // if (get_mag) // TODO For new magnetometer
        // {
        //     bmi270_h.get_bmm150_data(&mag_data);
        //     mag_data = mag_data - CONFIG.mag_bias;
        //     mag_data = mag_data * CONFIG.mag_scale;
        // }

        { // Add new reading to end of buffer, and remove oldest reading from start of buffer.
            std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
            arwain::Buffers::IMU_BUFFER.pop_front();
            arwain::Buffers::IMU_BUFFER.push_back({accel_data.x, accel_data.y, accel_data.z, gyro_data.x, gyro_data.y, gyro_data.z});
        }

        // Buffer mag_data if collected. TODO Is there actually any reason to buffer this?
        if (get_mag)
        {
            std::lock_guard<std::mutex> lock{arwain::Locks::MAG_BUFFER_LOCK};
            arwain::Buffers::MAG_BUFFER.pop_front();
            arwain::Buffers::MAG_BUFFER.push_back({mag_data.x, mag_data.y, mag_data.z});
        }

        // Log IMU to file.
        if (arwain::config.log_to_file)
        {
            acce_file << timeCount << " " << accel_data.x << " " << accel_data.y << " " << accel_data.z << "\n";
            gyro_file << timeCount << " " << gyro_data.x << " " << gyro_data.y << " " << gyro_data.z << "\n";
            // if (CONFIG.log_magnetometer)
            // {
            //     mag_file << timeCount << " " << mag_data.x << " " << mag_data.y << " " << mag_data.z << "\n";
            // }
        }

        // Perform an orientation filter step.
        if (arwain::config.use_magnetometer)
        {
            filter->update(
                timeCount,
                gyro_data.x, gyro_data.y, gyro_data.z,
                accel_data.x, accel_data.y, accel_data.z,
                mag_data.x, mag_data.y, mag_data.z
            );
        }
        else
        {
            filter->update(
                timeCount,
                gyro_data.x, gyro_data.y, gyro_data.z,
                accel_data.x, accel_data.y, accel_data.z
            );
        }

        // Extract Euler orientation from filter.
        euler_data.roll = filter->getRoll();
        euler_data.pitch = filter->getPitch();
        euler_data.yaw = filter->getYaw();

        // Extract quaternion orientation from filter.
        quat_data.w = filter->getW();
        quat_data.x = filter->getX();
        quat_data.y = filter->getY();
        quat_data.z = filter->getZ();

        { // Add orientation information to buffers.
            std::lock_guard<std::mutex> lock{arwain::Locks::ORIENTATION_BUFFER_LOCK};
            arwain::Buffers::EULER_ORIENTATION_BUFFER.pop_front();
            arwain::Buffers::EULER_ORIENTATION_BUFFER.push_back(euler_data);
            arwain::Buffers::QUAT_ORIENTATION_BUFFER.pop_front();
            arwain::Buffers::QUAT_ORIENTATION_BUFFER.push_back(quat_data);
        }

        // Add world-aligned IMU to its own buffer.
        world_accel_data = world_align(accel_data, quat_data);
        world_gyro_data = world_align(gyro_data, quat_data);
        {
            std::lock_guard<std::mutex> lock{arwain::Locks::IMU_BUFFER_LOCK};
            arwain::Buffers::IMU_WORLD_BUFFER.pop_front();
            arwain::Buffers::IMU_WORLD_BUFFER.push_back({
                world_accel_data.x, world_accel_data.y, world_accel_data.z,
                world_gyro_data.x, world_gyro_data.y, world_gyro_data.z
            });
        }

        // Add world-aligned magnetic field to buffer. TODO Why?
        if (get_mag)
        {
            world_mag_data = world_align(mag_data, quat_data);
            std::lock_guard<std::mutex> lock{arwain::Locks::MAG_BUFFER_LOCK};
            arwain::Buffers::MAG_WORLD_BUFFER.pop_front();
            arwain::Buffers::MAG_WORLD_BUFFER.push_back({
                world_mag_data.x, world_mag_data.y, world_mag_data.z
            });
        }

        // Log orientation information to file.
        if (arwain::config.log_to_file)
        {
            euler_file << timeCount << " " << euler_data.roll << " " << euler_data.pitch << " " << euler_data.yaw << "\n";
            quat_file << timeCount << " " << quat_data.w << " " << quat_data.x << " " << quat_data.y << " " << quat_data.z << "\n";
        }

        // Wait until the next tick.
        count++;
        loopTime = loopTime + interval;
        std::this_thread::sleep_until(loopTime);
    }

    delete filter;

    // Close all file handles.
    if (arwain::config.log_to_file)
    {
        acce_file.close();
        gyro_file.close();
        euler_file.close();
        quat_file.close();
        mag_file.close();
    }
}
#else
void imu_reader()
{
    ////////////////////////////////////////////////
    // COMPLETELY UNTESTED, DO NOT ATTEMPT TO USE //
    ////////////////////////////////////////////////

    // Quit immediately if IMU not enabled.
    if (NO_IMU)
    {
        return;
    }

    // Choose an orientation filter depending on configuration, with Madgwick as default.
    arwain::Filter* filter;
    if (CONFIG.orientation_filter == "efaroe")
    {
        filter = new arwain::eFaroe{quaternion{0,0,0,0}, CONFIG.gyro_bias, 100, CONFIG.efaroe_beta, CONFIG.efaroe_zeta};
    }
    else
    {
        filter = new arwain::Madgwick{1000.0/arwain::Intervals::IMU_READING_INTERVAL, CONFIG.madgwick_beta};
    }

    int count = 0;
    int get_mag = 0;

    // Local buffers for IMU data.
    vector3 accel_previous{0, 0, 0};  // Previous -> Data from the start of the previous loop
    vector3 accel_mid{0, 0, 0};       // Mid      -> Data halfway between the start of current loop and previous loop
    vector3 accel_now{0, 0, 0};       // Now      -> Data from the start of the current loop
    vector3 gyro_previous{0, 0, 0};
    vector3 gyro_mid{0, 0, 0};
    vector3 gyro_now{0, 0, 0};
    vector3 magnet_now{0, 0, 0};
    quaternion quaternion_now;
    quaternion quaternion_mid;

    // File handles for logging.
    arwain::Logger acce_file;
    arwain::Logger gyro_file;
    arwain::Logger mag_file;
    arwain::Logger quat_file;

    if (LOG_TO_FILE)
    {
        // Open file handles for data logging.
        acce_file.open(arwain::folder_date_string + "/acce.txt");
        gyro_file.open(arwain::folder_date_string + "/gyro.txt");
        mag_file.open(arwain::folder_date_string + "/mag.txt");
        quat_file.open(arwain::folder_date_string + "/game_rv.txt");

        // File headers
        acce_file << "# time x y z" << "\n";
        gyro_file << "# time x y z" << "\n";
        mag_file << "# time x y z" << "\n";
        quat_file << "# time w x y z" << "\n";
    }

    // Set up timing.
    auto timePrevious = std::chrono::system_clock::now();
    auto timeNow = std::chrono::system_clock::now();
    auto timeMid = std::chrono::system_clock::now();
    auto loopTime = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{arwain::Intervals::IMU_READING_INTERVAL * 2};

    while (!SHUTDOWN)
    {
        // Whether or not to get magnetometer on this spin.
        get_mag = ((count % 20 == 0) && (CONFIG.use_magnetometer || CONFIG.log_magnetometer));

        // Get the current time, and the midpoint in time between now and the start of the previous spin.
        timeNow = std::chrono::system_clock::now();
        timeMid = timeNow - (timeNow - timePrevious)/2;

        auto timeMidCount = timeMidCount;
        auto timeNowCount = timeNowCount;

        // Read sensor data and apply bias corrections.
        get_bmi270_data(&accel_now, &gyro_now);
        if (get_mag)
        {
            get_bmm150_data(&magnet_now);
            magnet_now = magnet_now - CONFIG.mag_bias;
            magnet_now = magnet_now * CONFIG.mag_scale;
        }
        accel_now = accel_now - CONFIG.accel_bias;
        gyro_now = gyro_now - CONFIG.gyro_bias;

        // Interpolate the accel and gyro readings halfway between the previous time step and now.
        accel_mid = (accel_now + accel_previous)/2.0;
        gyro_mid = (gyro_now + gyro_previous)/2.0;

        { // Add new readings to end of IMU buffer, and remove oldest two from start of buffer.
            std::lock_guard<std::mutex> lock{IMU_BUFFER_LOCK};
            IMU_BUFFER.pop_front();
            IMU_BUFFER.pop_front();
            IMU_BUFFER.push_back({accel_mid.x, accel_mid.y, accel_mid.z, gyro_mid.x, gyro_mid.y, gyro_mid.z});
            IMU_BUFFER.push_back({accel_now.x, accel_now.y, accel_now.z, gyro_now.x, gyro_now.y, gyro_now.z});
        }

        if (LOG_TO_FILE)
        { // Log IMU to file.
            acce_file << timeMidCount << " " << accel_mid.x << " " << accel_mid.y << " " << accel_mid.z << "\n";
            gyro_file << timeMidCount << " " << gyro_mid.x << " " << gyro_mid.y << " " << gyro_mid.z << "\n";
            acce_file << timeNowCount << " " << accel_now.x << " " << accel_now.y << " " << accel_now.z << "\n";
            gyro_file << timeNowCount << " " << gyro_now.x << " " << gyro_now.y << " " << gyro_now.z << "\n";
            if (CONFIG.log_magnetometer)
            { // Log magnetometer to file.
                mag_file << timeNowCount << " " << magnet_now.x << " " << magnet_now.y << " " << magnet_now.z << "\n";
            }
        }

        // Update orientation filter twice, using interpolated sample and current sample.
        if (CONFIG.use_magnetometer)
        {
            filter->update(timeMidCount, gyro_mid.x, gyro_mid.y, gyro_mid.z, accel_mid.x, accel_mid.y, accel_mid.z, magnet_now.z, magnet_now.y, magnet_now.z);
            quaternion_mid = {filter->getW(),filter->getX(),filter->getY(),filter->getZ()};
            filter->update(timeNowCount, gyro_now.x, gyro_now.y, gyro_now.z, accel_now.x, accel_now.y, accel_now.z, magnet_now.z, magnet_now.y, magnet_now.z);
            quaternion_now = {filter->getW(),filter->getX(),filter->getY(),filter->getZ()};
        }
        else
        {
            filter->update(timeMidCount, gyro_mid.x, gyro_mid.y, gyro_mid.z, accel_mid.x, accel_mid.y, accel_mid.z);
            quaternion_mid = {filter->getW(),filter->getX(),filter->getY(),filter->getZ()};
            filter->update(timeNowCount, gyro_now.x, gyro_now.y, gyro_now.z, accel_now.x, accel_now.y, accel_now.z);
            quaternion_now = {filter->getW(),filter->getX(),filter->getY(),filter->getZ()};
        }

        { // Add orientation to buffer and remove oldest two samples.
            std::lock_guard<std::mutex> lock{ORIENTATION_BUFFER_LOCK};
            QUAT_ORIENTATION_BUFFER.pop_front();
            QUAT_ORIENTATION_BUFFER.pop_front();
            QUAT_ORIENTATION_BUFFER.push_back(quaternion_mid);
            QUAT_ORIENTATION_BUFFER.push_back(quaternion_now);
        }

        // World-align IMU data and add to its own buffer.
        auto world_accel_mid = world_align(accel_mid, quaternion_mid);
        auto world_accel = world_align(accel_now, quaternion_now);
        auto world_gyro_mid = world_align(gyro_mid, quaternion_mid);
        auto world_gyro = world_align(gyro_now, quaternion_now);
        {
            std::lock_guard<std::mutex> lock{IMU_BUFFER_LOCK};
            IMU_WORLD_BUFFER.pop_front();
            IMU_WORLD_BUFFER.pop_front();
            IMU_WORLD_BUFFER.push_back({world_accel_mid.x, world_accel_mid.y, world_accel_mid.z, world_gyro_mid.x, world_gyro_mid.y, world_gyro_mid.z});
            IMU_WORLD_BUFFER.push_back({world_accel.x, world_accel.y, world_accel.z, world_gyro.x, world_gyro.y, world_gyro.z});
        }

        // Log orientation information to file.
        if (LOG_TO_FILE)
        {
            quat_file << timeMidCount << " " << quaternion_mid.w << " " << quaternion_mid.x << " " << quaternion_mid.y << " " << quaternion_mid.z << "\n";
            quat_file << timeNowCount << " " << quaternion_now.w << " " << quaternion_now.x << " " << quaternion_now.y << " " << quaternion_now.z << "\n";
        }

        // Store last reading and wait until next tick.
        accel_previous = accel_now;
        gyro_previous = gyro_now;
        timePrevious = timeNow;
        count += 2;
        loopTime = loopTime + interval;
        std::this_thread::sleep_until(loopTime);
    }

    delete filter;

    // Close file handles.
    if (LOG_TO_FILE)
    {
        acce_file.close();
        gyro_file.close();
        quat_file.close();
        mag_file.close();
    }
}
#endif