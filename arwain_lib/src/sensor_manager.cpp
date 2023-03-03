// TODO Error checking when starting and using IMUs.

#include <mutex>
#include <deque>
#include <string>
#include <cmath>
#include <thread>
#include <functional>

#include "arwain_utils.hpp"
#include "sensor_manager.hpp"
#include "multi_imu.hpp"
#include "madgwick.hpp"
#include "efaroe.hpp"
#include "logger.hpp"
#include "timers.hpp"
#include "lis3mdl.hpp"
#include "calibration.hpp"
#include "arwain_thread.hpp"
#include "exceptions.hpp"

/** \brief Rotates a Vector3 according to a quaternion orientation.
 * \param vec The Vector3 to rotate.
 * \param orientation The rotation to apply to the Vector3.
 * \return Rotated Vector3.
 */
Vector3 SensorManager::world_align(const Vector3& vec, const Quaternion& rotation)
{
    return arwain::apply_quat_rotor_to_vector3(vec, rotation);
}

/** \brief The main job thread. Executes a specific job thread when in appropriate mode, or does sleep if in non-relevant mode. */
void SensorManager::run()
{
    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
        {
            case arwain::OperatingMode::GyroscopeCalibration:
                run_gyro_calibration();
                break;
            case arwain::OperatingMode::MagnetometerCalibration:
                run_magn_calibration();
                break;
            case arwain::OperatingMode::AccelerometerCalibration:
                run_accel_calibration();
                break;
            case arwain::OperatingMode::Inference:
                run_inference();
                break;
            case arwain::OperatingMode::Idle:
                run_idle();
                break;
            case arwain::OperatingMode::TestStanceDetector:
                run_test_stance_detector();
                break;
            case arwain::OperatingMode::SelfTest:
                run_self_test();
                break;
            default:
                sleep_ms(10);
                break;
        }
    }
}

/** \brief Runs a calibration pass on the gyroscope for the given IMU. Runtime bias parameters are
 * updated for the IMU, and the results are written to the arwain configuration file. This
 * function requires that autocalibration is disabled for the IMU in question.
 * \param imu The IMU device with the gyroscope to be calibrated.
 * \return Will return false if autocalibration is turned onm, and calibration could not be
 * performed. Return true if calibration is performed.
 */
bool calibrate_gyroscope_bias(IIM42652<I2CDEVICEDRIVER>& imu)
{
    if (imu.auto_calib_enabled())
    {
        return false;
    }

    std::cout << "Calibrating gyro on I2C: " << imu.get_bus() << " " << imu.get_address() << std::endl;

    // Create a calibrator and feed in gyroscope readings until it converges.    
    GyroscopeCalibrator calibrator;
    while (!calibrator.is_converged())
    {
        calibrator.feed(imu.read_IMU().gyro);
    }
    Vector3 gyroscope_bias = calibrator.get_params();

    arwain::config.gyro1_bias = gyroscope_bias;
    arwain::config.replace("gyro1_bias_x", gyroscope_bias.x);
    arwain::config.replace("gyro1_bias_y", gyroscope_bias.y);
    arwain::config.replace("gyro1_bias_z", gyroscope_bias.z);
    
    imu.set_gyro_bias(
        arwain::config.gyro1_bias.x,
        arwain::config.gyro1_bias.y,
        arwain::config.gyro1_bias.z
    );
    
    std::cout << "Calibration of gyro complete" << std::endl;

    return true;
}

/** \brief Runs a gyroscope calibration on all active IMUs.
 * \exception Can throw a runtime error if gyro autocalibration is turned on while calibration is running.
*/
void SensorManager::run_gyro_calibration()
{
    Vector3 results;

    imu1.disable_auto_calib();
    imu2.disable_auto_calib();
    imu3.disable_auto_calib();
    
    imu1.set_gyro_bias(0, 0, 0);
    imu2.set_gyro_bias(0, 0, 0);
    imu3.set_gyro_bias(0, 0, 0);
    
    bool success = calibrate_gyroscope_bias(imu1);
    success |= calibrate_gyroscope_bias(imu2);
    success |= calibrate_gyroscope_bias(imu3);

    // The ARWAIN application uses several threads. It is possible in principle for the autocalibration state
    // to be changed before active calibration has finished. This should not be allowed as invalid calibration
    // parameters will result and all gyroscope readings will be suspect.
    if (!success)
    {
        throw std::runtime_error{"Gyroscope calibration could not be performed because autocalibration was not disabled."};
    }

    // Reset calibration parameters and re-enable autocalibration.
    imu1.enable_auto_calib();
    imu2.enable_auto_calib();
    imu3.enable_auto_calib();

    // Callback to be executed when active gyroscope calibration is complete.
    if (post_gyro_calib_callback)
    {
        post_gyro_calib_callback();
    }
}

void SensorManager::run_magn_calibration()
{
    // Unset current calibration parameters.
    magnetometer.set_calibration_parameters(
        {0, 0, 0},
        {1, 1, 1},
        {0, 0, 0}
    );

    // Perform magnetometer calibration procedure.
    MagnetometerCalibrator calibrator;
    std::cout << "About to start magnetometer calibration" << std::endl;
    std::cout << "Randomly move the device through all orientations" << std::endl;
    sleep_ms(1000);
    std::cout << "Calibration started ..." << std::endl;

    while (calibrator.get_sphere_coverage_quality() < 60)
    {
        calibrator.feed(magnetometer.read());
        sleep_ms(100);
    }
    auto [bias, scale] = calibrator.solve();

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
    magnetometer.set_calibration_parameters(
        arwain::config.mag_bias,
        arwain::config.mag_scale,
        {arwain::config.mag_scale_xy, arwain::config.mag_scale_yz, arwain::config.mag_scale_xz}
    );

    std::cout << "Magnetometer calibration complete" << std::endl;

    if (post_gyro_calib_callback)
    {
        post_gyro_calib_callback();
    }
}

void SensorManager::run_accel_calibration()
{
    // TODO
    throw NotImplemented{__FUNCTION__};
}

void SensorManager::run_idle()
{
    Timers::IntervalTimer<std::chrono::milliseconds> loop_scheduler{arwain::Intervals::IMU_READING_INTERVAL, "arwain_sensor_manager_run_idle"};

    while (arwain::system_mode == arwain::OperatingMode::Idle || arwain::system_mode == arwain::OperatingMode::TestStanceDetector)
    {
        int64_t time_count = loop_scheduler.count();

        auto [accel_data1, gyro_data1] = imu1.read_IMU();
        // Force approximate alignment of the IMUs.

        // Feed the activity metric.
        arwain::activity_metric.feed_acce(accel_data1);
        arwain::activity_metric.feed_gyro(gyro_data1);

        Vector3 magnet = magnetometer.read();
        magnet = {magnet.y, magnet.x, magnet.z}; // align magnetometer with IMU.

        arwain::Buffers::IMU_BUFFER.push_back({accel_data1, gyro_data1});

        madgwick_filter_1.update(time_count, gyro_data1.x, gyro_data1.y, gyro_data1.z, accel_data1.x, accel_data1.y, accel_data1.z);
        madgwick_filter_mag_1.update(time_count, gyro_data1.x, gyro_data1.y, gyro_data1.z, accel_data1.x, accel_data1.y, accel_data1.z, magnet.x, magnet.y, magnet.z);

        // Extract Euler orientation from filters.
        Quaternion madgwick_quaternion_data1 = {madgwick_filter_1.get_w(), madgwick_filter_1.get_x(), madgwick_filter_1.get_y(), madgwick_filter_1.get_z()};
        Quaternion madgwick_quaternion_mag_data1 = {madgwick_filter_mag_1.get_w(), madgwick_filter_mag_1.get_x(), madgwick_filter_mag_1.get_y(), madgwick_filter_mag_1.get_z()};
        [[maybe_unused]] EulerOrientation madgwick_euler_mag_data1 = arwain::compute_euler(madgwick_quaternion_mag_data1);

        arwain::Buffers::QUAT_ORIENTATION_BUFFER.push_back(madgwick_quaternion_data1);

        // Add world-aligned IMU to its own buffer.
        Vector3 world_accel_data1 = world_align(accel_data1, madgwick_quaternion_data1);
        Vector3 world_gyro_data1 = world_align(gyro_data1, madgwick_quaternion_data1);
        arwain::Buffers::IMU_WORLD_BUFFER.push_back({world_accel_data1, world_gyro_data1});
        arwain::rolling_average_accel_z_for_altimeter.feed(world_accel_data1.z);

        // Compute the new correction based on magnetometer/gyro filter diffs.
        double new_yaw_offset = unwrap_phase_radians(madgwick_filter_mag_1.get_yaw_radians() - madgwick_filter_1.get_yaw_radians(), arwain::yaw_offset);
        if (arwain::yaw_offset == 0)
        {
            arwain::yaw_offset = new_yaw_offset;
        }
        else
        {
            arwain::yaw_offset = new_yaw_offset*0.001 + 0.999*arwain::yaw_offset;
        }

        // Wait until the next tick.
        loop_scheduler.await();
    }
}

void SensorManager::run_test_stance_detector()
{
    run_idle();
}

void SensorManager::run_self_test()
{
    // TODO
    throw NotImplemented{__FUNCTION__};
}

/** \brief Executes the inference mode job thread. */
void SensorManager::run_inference()
{
    setup_inference();

    // Set up timing.
    Timers::IntervalTimer<std::chrono::milliseconds> loop_scheduler{arwain::Intervals::IMU_READING_INTERVAL, "arwain_sensor_manager_run_infer"};

    while (arwain::system_mode == arwain::OperatingMode::Inference)
    {
        int64_t time_count = loop_scheduler.count(); // Provides an accurate count of milliseconds passed since last loop iteration.

        auto [accel_data1, gyro_data1] = imu1.read_IMU();

        Vector3 magnet = magnetometer.read();
        magnet = {magnet.y, magnet.x, magnet.z}; // align magnetometer with IMU.

        arwain::Buffers::IMU_BUFFER.push_back({accel_data1, gyro_data1});

        madgwick_filter_1.update(time_count, gyro_data1.x, gyro_data1.y, gyro_data1.z, accel_data1.x, accel_data1.y, accel_data1.z);
        madgwick_filter_mag_1.update(time_count, gyro_data1.x, gyro_data1.y, gyro_data1.z, accel_data1.x, accel_data1.y, accel_data1.z, magnet.x, magnet.y, magnet.z);

        // Extract Euler orientation from filters.
        Quaternion madgwick_quaternion_data1 = {madgwick_filter_1.get_w(), madgwick_filter_1.get_x(), madgwick_filter_1.get_y(), madgwick_filter_1.get_z()};
        Quaternion madgwick_quaternion_mag_data1 = {madgwick_filter_mag_1.get_w(), madgwick_filter_mag_1.get_x(), madgwick_filter_mag_1.get_y(), madgwick_filter_mag_1.get_z()};
        EulerOrientation madgwick_euler_data1 = arwain::compute_euler(madgwick_quaternion_data1);
        EulerOrientation madgwick_euler_mag_data1 = arwain::compute_euler(madgwick_quaternion_mag_data1);

        arwain::Buffers::EULER_ORIENTATION_BUFFER.push_back(madgwick_euler_data1);
        arwain::Buffers::QUAT_ORIENTATION_BUFFER.push_back(madgwick_quaternion_data1);

        // Add world-aligned IMU to its own buffer.
        Vector3 world_accel_data1 = world_align(accel_data1, madgwick_quaternion_data1);
        Vector3 world_gyro_data1 = world_align(gyro_data1, madgwick_quaternion_data1);
        arwain::Buffers::IMU_WORLD_BUFFER.push_back({world_accel_data1, world_gyro_data1});
        arwain::rolling_average_accel_z_for_altimeter.feed(world_accel_data1.z);

        // Write all log files.
        acce_file_1 << time_count << " " << accel_data1.x << " " << accel_data1.y << " " << accel_data1.z << "\n";
        gyro_file_1 << time_count << " " << gyro_data1.x << " " << gyro_data1.y << " " << gyro_data1.z << "\n";
        world_acce_file_1 << time_count << " " << world_accel_data1.x << " " << world_accel_data1.y << " " << world_accel_data1.z << "\n";
        world_gyro_file_1 << time_count << " " << world_gyro_data1.x << " " << world_gyro_data1.y << " " << world_gyro_data1.z << "\n";
        madgwick_euler_file_1 << time_count << " " << madgwick_euler_data1.roll << " " << madgwick_euler_data1.pitch << " " << madgwick_euler_data1.yaw << "\n";
        madgwick_euler_mag_file_1 << time_count << " " << madgwick_euler_mag_data1.roll << " " << madgwick_euler_mag_data1.pitch << " " << madgwick_euler_mag_data1.yaw << "\n";
        madgwick_quat_file_1 << time_count << " " << madgwick_quaternion_data1.w << " " << madgwick_quaternion_data1.x << " " << madgwick_quaternion_data1.y << " " << madgwick_quaternion_data1.z << "\n";
        imu_calib_file_1 << time_count << " " << imu1.get_gyro_calib_x() << " " << imu1.get_gyro_calib_y() << " " << imu1.get_gyro_calib_z() << "\n";

        // Wait until the next tick.
        loop_scheduler.await();
    }

    cleanup_inference();
}

/** \brief Opens file handles for arwain::Logger and writes appropriate headers to the files. */
void SensorManager::setup_inference()
{
    // Open file handles.
    ori_diff_file.open(arwain::folder_date_string + "/ori_diff.txt");
    acce_file_1.open(arwain::folder_date_string + "/acce.txt");
    world_acce_file_1.open(arwain::folder_date_string + "/world_acce.txt");
    gyro_file_1.open(arwain::folder_date_string + "/gyro.txt");
    world_gyro_file_1.open(arwain::folder_date_string + "/world_gyro.txt");
    madgwick_euler_file_1.open(arwain::folder_date_string + "/madgwick_euler_orientation_1.txt");
    madgwick_quat_file_1.open(arwain::folder_date_string + "/madgwick_game_rv_1.txt");
    madgwick_euler_mag_file_1.open(arwain::folder_date_string + "/madgwick_mag_euler_orientation_1.txt");
    imu_calib_file_1.open(arwain::folder_date_string + "/imu_calib_1.txt");
    imu_calib_file_2.open(arwain::folder_date_string + "/imu_calib_2.txt");
    imu_calib_file_3.open(arwain::folder_date_string + "/imu_calib_3.txt");

    // File headers
    ori_diff_file << "time yaw" << "\n";
    acce_file_1 << "time x y z" << "\n";
    world_acce_file_1 << "time x y z" << "\n";
    gyro_file_1 << "time x y z" << "\n";
    world_gyro_file_1 << "time x y z" << "\n";
    madgwick_euler_file_1 << "time roll pitch yaw" << "\n";
    madgwick_quat_file_1 << "time w x y z" << "\n";
    madgwick_euler_mag_file_1 << "time roll pitch yaw" << "\n";
    imu_calib_file_1 << "time gx_bias gy_bias gz_bias" << "\n";
    imu_calib_file_2 << "time gx_bias gy_bias gz_bias" << "\n";
    imu_calib_file_3 << "time gx_bias gy_bias gz_bias" << "\n";
}

/** \brief Closes file handles for arwain::Logger objects. */
arwain::ReturnCode SensorManager::cleanup_inference()
{
    bool closed_files = true;

    closed_files |= ori_diff_file.close();
    closed_files |= acce_file_1.close();
    closed_files |= world_acce_file_1.close();
    closed_files |= gyro_file_1.close();
    closed_files |= world_gyro_file_1.close();
    closed_files |= madgwick_euler_file_1.close();
    closed_files |= madgwick_quat_file_1.close();
    closed_files |= madgwick_euler_mag_file_1.close();
    closed_files |= imu_calib_file_1.close();
    closed_files |= imu_calib_file_2.close();
    closed_files |= imu_calib_file_3.close();

    if (closed_files)
    {
        return arwain::ReturnCode::Success;
    }
    else
    {
        return arwain::ReturnCode::IOError;
    }
}

void SensorManager::core_setup()
{
    // Configure IMUs.
    imu1 = IIM42652<I2CDEVICEDRIVER>{arwain::config.imu1_address, arwain::config.imu1_bus};
    imu1.set_gyro_bias(arwain::config.gyro1_bias.x, arwain::config.gyro1_bias.y, arwain::config.gyro1_bias.z);
    imu1.set_accel_bias(arwain::config.accel1_bias.x, arwain::config.accel1_bias.y, arwain::config.accel1_bias.z);
    imu1.set_accel_scale(arwain::config.accel1_scale.x, arwain::config.accel1_scale.y, arwain::config.accel1_scale.z);
    imu1.enable_auto_calib();

    imu2 = IIM42652<I2CDEVICEDRIVER>{arwain::config.imu2_address, arwain::config.imu2_bus};
    imu2.set_gyro_bias(arwain::config.gyro2_bias.x, arwain::config.gyro2_bias.y, arwain::config.gyro2_bias.z);
    imu2.set_accel_bias(arwain::config.accel2_bias.x, arwain::config.accel2_bias.y, arwain::config.accel2_bias.z);
    imu2.set_accel_scale(arwain::config.accel2_scale.x, arwain::config.accel2_scale.y, arwain::config.accel2_scale.z);
    imu2.enable_auto_calib();

    imu3 = IIM42652<I2CDEVICEDRIVER>{arwain::config.imu3_address, arwain::config.imu3_bus};
    imu3.set_gyro_bias(arwain::config.gyro3_bias.x, arwain::config.gyro3_bias.y, arwain::config.gyro3_bias.z);
    imu3.set_accel_bias(arwain::config.accel3_bias.x, arwain::config.accel3_bias.y, arwain::config.accel3_bias.z);
    imu3.set_accel_scale(arwain::config.accel3_scale.x, arwain::config.accel3_scale.y, arwain::config.accel3_scale.z);
    imu3.enable_auto_calib();

    // Configure magnetometer.
    magnetometer = LIS3MDL<I2CDEVICEDRIVER>{arwain::config.magn_address, arwain::config.magn_bus};
    magnetometer.set_calibration_parameters(
        arwain::config.mag_bias,
        arwain::config.mag_scale,
        {arwain::config.mag_scale_xy, arwain::config.mag_scale_yz, arwain::config.mag_scale_xz}
    );

    // Choose an orientation filter depending on configuration, with Madgwick as default.
    madgwick_filter_1 = arwain::Madgwick{1000.0/static_cast<double>(arwain::Intervals::IMU_READING_INTERVAL), 0.1};
    madgwick_filter_mag_1 = arwain::Madgwick{1000.0/static_cast<double>(arwain::Intervals::IMU_READING_INTERVAL), arwain::config.madgwick_beta_conv};
    
    // After the specificed period has passed, set the magnetic filter gain to the standard value.
    // This thread is joined when the namespace is joined, to simulate a destructor cleanup.
    // TODO Note that this may not be thread safe; if the program exists before this thread finishes, I'm not sure of destruction order. Investigate.
    quick_madgwick_convergence_thread = ArwainThread{
        [this]()
        {
            std::this_thread::sleep_for(std::chrono::milliseconds{5000});
            this->madgwick_filter_mag_1.set_beta(arwain::config.madgwick_beta);
        }
    };
}

SensorManager::SensorManager()
{
    init();
}

/** \brief Does overall initialization and sets up the job thread. */
bool SensorManager::init()
{
    if (arwain::config.no_imu)
    {
        return false;
    }
    core_setup();
    job_thread = ArwainThread{&SensorManager::run, "arwain_imu_th", this};
    return true;
}

void SensorManager::set_post_gyro_calibration_callback(std::function<void()> func)
{
    post_gyro_calib_callback = func;
}

/** \brief Block until the job thread can be joined. */
void SensorManager::join()
{
    if (job_thread.joinable())
    {
        job_thread.join();
    }
    if (quick_madgwick_convergence_thread.joinable())
    {
        quick_madgwick_convergence_thread.join();
    }
}
