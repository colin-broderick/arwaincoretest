#include <thread>
#include <chrono>

#include "build_config.hpp"
#include "arwain.hpp"
#include "arwain_tests.hpp"
#include "bmp384.hpp"
#include "IMU_IIM42652_driver.hpp"
#include "lis3mdl.hpp"
#include "madgwick.hpp"

#if USE_ROS == 1
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#endif

#if USE_UUBLA == 1
#include "uubla.hpp"
#endif

static euler_orientation_t computer_euler_degrees(Quaternion& q)
{
    euler_orientation_t euler;
    euler.roll = std::atan2(q.w*q.x + q.y*q.z, 0.5f - q.x*q.x - q.y*q.y)  * 180.0 / 3.14159;
	euler.pitch = std::asin(-2.0 * (q.x*q.z - q.w*q.y))  * 180.0 / 3.14159;
	euler.yaw = std::atan2(q.x*q.y + q.w*q.z, 0.5 - q.y*q.y - q.z*q.z)  * 180.0 / 3.14159;
    return euler;
}

#if USE_UUBLA == 1
int arwain::test_uubla_integration()
{
    std::cout << "Creating UUBLA network and running for 20 s" << "\n";

    // Create and configure the UUBLA network.
    UUBLA::Network uubla{
        arwain::config.uubla_serial_port,
        arwain::config.uubla_baud_rate
    };
    uubla.configure("force_z_zero", true);
    uubla.configure("ewma_gain", 0.1);

    // Add test callbacks for adding/removal of nodes.
    auto add = [](const std::string& node_name) { std::cout << "Added " << node_name << "\n"; };
    auto rem = [](const std::string& node_name) { std::cout << "Removed " << node_name << "\n"; };
    uubla.add_node_callback = add;
    uubla.remove_node_callback = rem;

    // Start the solver and run for 20 seconds.
    uubla.start_reading();
    std::thread solver_th{solver_fn, &uubla};
    for (int i = 0; i < 200; i++)
    {
        sleep_ms(100);
    }

    // Cleanup and exit.
    std::cout << "Stopping UUBLA network" << "\n";
    uubla.stop();
    solver_th.join();

    return arwain::ExitCodes::Success;
}
#endif

int arwain::test_pressure()
{
    BMP384 bmp384{arwain::config.pressure_address, arwain::config.pressure_bus};

    // Set up timing.
    auto loopTime = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{50};

    double altitude = 100;
    double factor = arwain::config.altitude_filter_weight;

    auto [pressure, temperature] = bmp384.read();
    pressure = pressure - arwain::config.pressure_offset;
    altitude = BMP384::calculate_altitude(pressure / 100.0, temperature, arwain::config.sea_level_pressure);

    sleep_ms(50);

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        auto [new_pressure, new_temperature] = bmp384.read();
        new_pressure = new_pressure - arwain::config.pressure_offset;
        double new_alt = BMP384::calculate_altitude(new_pressure / 100.0, new_temperature, arwain::config.sea_level_pressure);
        altitude = factor * altitude + (1.0 - factor) * new_alt;

        std::cout << "Pressure:    " << new_pressure / 100.0 << " hPa" << std::endl;
        std::cout << "Temperature: " << new_temperature << " \u00B0C" << std::endl;
        std::cout << "Altitude:    " << altitude << " m; " << new_alt << " m" << std::endl;
        std::cout << std::endl;

        // Wait until next tick.
        loopTime = loopTime + interval;
        std::this_thread::sleep_until(loopTime);
    }

    return arwain::ExitCodes::Success;
}

/** \brief Utility functional for checking that the IMU is operational.
 */
int arwain::test_imu()
{
    // Initialize the IMU.
    IMU_IIM42652 imu{0x68, "/dev/i2c-1"};
    imu.set_accel_bias(arwain::config.accel1_bias.x, arwain::config.accel1_bias.y, arwain::config.accel1_bias.z);
    imu.set_accel_scale(arwain::config.accel1_scale.x, arwain::config.accel1_scale.y, arwain::config.accel1_scale.z);
    imu.set_gyro_bias(arwain::config.gyro1_bias.x, arwain::config.gyro1_bias.y, arwain::config.gyro1_bias.z);

    // Set up timing.
    auto time = std::chrono::system_clock::now();
    std::chrono::milliseconds interval{10};

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        auto [accel_data, gyro_data] = imu.read_IMU();
        accel_data.x = (accel_data.x - arwain::config.accel1_bias.x) * arwain::config.accel1_scale.x;
        accel_data.y = (accel_data.y - arwain::config.accel1_bias.y) * arwain::config.accel1_scale.y;
        accel_data.z = (accel_data.z - arwain::config.accel1_bias.z) * arwain::config.accel1_scale.z;

        // Display IMU data.
        std::cout << time.time_since_epoch().count() << std::fixed << std::right << std::setprecision(3) << "\t" << accel_data.x << "\t" << accel_data.y << "\t" << accel_data.z << "\t" << gyro_data.x << "\t" << gyro_data.y << "\t" << gyro_data.z << "\n";

        // Wait until the next tick.
        time = time + interval;
        std::this_thread::sleep_until(time);

    }

    return arwain::ExitCodes::Success;
}

int arwain::test_lora_rx()
{
    LoRa lora{arwain::config.lora_address, true};

    if (lora.test_chip() == 0x1A)
    {
        std::cout << "Found chip" << std::endl;
    };

    std::cout << "Receiving LoRa messages ..." << std::endl;

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        auto [rx, message] = lora.receive_string(1000);
        if (rx)
        {
            std::cout << message << std::endl;
        }
    }

    return arwain::ExitCodes::Success;
}

#if USE_ROS == 1
int arwain::test_mag(int argc, char **argv)
{
    ros::NodeHandle nh;
    auto pub = nh.advertise<geometry_msgs::Vector3Stamped>("/imu/mag", 1000);

    LIS3MDL magn{arwain::config.magn_address, arwain::config.magn_bus};

    while (arwain::system_mode != arwain::OperatingMode::Terminate && ros::ok())
    {
        Vector3 reading = magn.read();
        geometry_msgs::Vector3Stamped msg;
        msg.header.stamp = ros::Time::now();
        msg.vector.x = reading.x;
        msg.vector.y = reading.y;
        msg.vector.z = reading.z;
        pub.publish(msg);
        sleep_ms(20);
    }

    return arwain::ExitCodes::Success;
}
#else
/** \brief Checks that the correct chip ID can be read from the magnetometer. If so, reads and prints orientation until interrupted. */
int arwain::test_mag()
{
    LIS3MDL magn{arwain::config.magn_address, arwain::config.magn_bus};
    magn.set_calibration(
        arwain::config.mag_bias,
        arwain::config.mag_scale,
        {arwain::config.mag_scale_xy, arwain::config.mag_scale_yz, arwain::config.mag_scale_xz}    
    );

    int id = magn.test_chip();
    if (id != 0x3D)
    {
        std::cout << "Chip ID incorrect: should be 0x3D, got " <<std::hex << std::showbase << id << std::endl;
        return arwain::ExitCodes::FailedMagnetometer;
    }
    std::cout << "Chip ID: " << std::hex << std::showbase << magn.test_chip() << std::dec << std::endl;

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        Vector3 reading = magn.read();
        std::cout << "Magnetometer readings: " << reading << " .... " << reading.magnitude() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }

    return arwain::ExitCodes::Success;
}
#endif

int arwain::test_lora_tx()
{
    LoRa lora{arwain::config.lora_address, false};

    if (lora.test_chip() == 0x1A)
    {
        std::cout << "Found chip" << std::endl;
    };

    std::string message = "ARWAIN.LoRa";
    std::cout << "Transmitting message \"" << message << ".x\" at 1 Hz ..." << std::endl;

    int i = 0;
    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        i++;
        std::string msg = message + "." + std::to_string(i);
        lora.send_message(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds{1000});
    }

    return arwain::ExitCodes::Success;
}

int arwain::test_ori(int frequency)
{
    IMU_IIM42652 imu{config.imu1_address, config.imu1_bus};
    imu.set_gyro_bias(arwain::config.gyro1_bias.x, arwain::config.gyro1_bias.y, arwain::config.gyro1_bias.z);
    imu.enable_auto_calib();
    arwain::Madgwick filter{static_cast<double>(frequency), config.madgwick_beta};
    // arwain::eFaroe filter{{1, 0, 0, 0}, config.gyro1_bias, 0, config.efaroe_beta, config.efaroe_zeta};

    auto time = std::chrono::high_resolution_clock::now();
    std::chrono::milliseconds interval{1000/frequency};
    int count = 0;
    euler_orientation_t euler;
    Quaternion quat;

    std::cout << "Starting orientation filter at " << frequency << " Hz" << std::endl;

    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        time += interval;
        auto timeCount = time.time_since_epoch().count();
        std::this_thread::sleep_until(time);

        auto [accel, gyro] = imu.read_IMU();
        accel = accel - config.accel1_bias;
        
        filter.update(timeCount, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);

        count++;
        if (count % frequency == 0)
        {
            quat = {filter.getW(), filter.getX(), filter.getY(), filter.getZ()};
            euler = computer_euler_degrees(quat);
            std::cout << "Quaternion: " << std::fixed << std::showpos << filter.getW() << " " << filter.getX() << " " << filter.getY() << " " << filter.getZ() << "\t\t";
            std::cout << "Euler: " << std::fixed << std::showpos << euler.roll << " " << euler.pitch << " " << euler.yaw << "\n";
        }
    }
    
    return arwain::ExitCodes::Success;
}
