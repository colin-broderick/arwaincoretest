#ifndef _ARWAIN_STANCE_DETECTION_HPP
#define _ARWAIN_STANCE_DETECTION_HPP

#include <vector>
#include <array>
#include <chrono>
#include <mutex>
#include <deque>

#include <arwain/vector3.hpp>
#include <arwain/quaternion.hpp>
#include <arwain/devices/iim42652.hpp>

#include "arwain/logger.hpp"
#include "arwain/thread.hpp"
#include "arwain/job_interface.hpp"

class StanceDetector
{
    public:
        enum Stance {
            Inactive,   // The wearer is upright and active but not moving anywhere.
            Prone,      // The wearer is horizontally oriented but not moving.
            Walking,    // The wearer is making steady progress at normal walking pace.
            Searching,  // The wearer shows high kinetic activity but forward progress is slow.
            Crawling,   // The wearer has horizontal attitude and is moving at a crawling pace.
            Running,    // The wearer is moving at greater than walking pace.
            Climbing    // The wearer is moving vertically; stairs or ladders.
        };
        enum FallState { 
            NotFalling,
            Falling     // There has recently been a period of rapid descent (~0 measured by accelerometers).
        };
        enum EntangleState { // ********** POSSIBLY DEPRECATED ********** //
            NotEntangled,   // The wearers activity does not indicate entanglement.
            Entangled       // The wearers activity indicates they may be entangled in cables.
        };
        enum Axis {
            XAxis,
            YAxis,
            ZAxis
        };
        enum Attitude {
            Vertical,       // The wearer is vertically oriented, i.e. stood upright.
            Horizontal      // The wearer is horizontally oriented, e.g. prone or crawling.
        };

    TESTABLE:
        double m_a_mean_magnitude;
        double m_g_mean_magnitude;
        double m_v_mean_magnitude;

        std::chrono::system_clock::time_point last_freefall_detection{};

        Vector3 m_accel_means;
        Vector3 m_speed_means;

        int m_vertical_axis = 1;
        int m_primary_axis;
        int m_speed_axis;

        int m_count = 0;
        double m_struggle = 0;

        // Status indicators.
        FallState m_falling = NotFalling;
        EntangleState m_entangled = NotEntangled;
        Attitude m_attitude = Vertical;
        Stance m_stance = Inactive;
        int m_climbing = 0;

        // Fall/entanglment thresholding parameters.
        double m_freefall_sensitivity;
        double m_struggle_threshold;
        double m_gravity;
        double m_a_twitch;
        double m_tmp_struggle;
        double m_sfactor = 1;
        double m_activity;
        std::vector<double> m_struggle_window;

        // Stance thresholding parameters.
        double m_climbing_threshold;
        double m_crawling_threshold;
        double m_running_threshold;
        double m_walking_threshold;
        double m_active_threshold;

        // Parameter access safety.
        std::mutex m_fall_lock;
        std::mutex m_stance_lock;

        // Utility methods.
        Axis biggest_axis(const Vector3 &arr);
        double activity(double a, double g, double v);
        double vector_mean(const std::vector<double> &values);
        double buffer_mean_magnitude(const std::vector<Vector3> &buffer);
        double buffer_mean_magnitude(const std::deque<Vector3> &buffer);
        Vector3 get_means(const std::vector<Vector3> &source_vector);
        Vector3 get_means(const std::deque<Vector3> &source_vector);
        std::array<double, 6> get_means(const std::deque<std::array<double, 6>> &source_vector);
        void check_for_falls(const std::deque<ImuData>& imu_data);
        void register_freefall_event();
    
    public:
        // Constructors.
        StanceDetector() = default;
        StanceDetector(double freefall_sensitivity, double crawling_threshold, double running_threshold, double walking_threshold, double active_threshold, double struggle_threshold);

        // General methods.
        void update_attitude(Quaternion rotation_quaternion);
        void run(const std::deque<ImuData> &imu_data, const std::deque<Vector3> &vel_data);
        void clear_freefall_flag();
        int seconds_since_last_freefall() const;

        // Getters.
        Stance get_stance();
        Attitude get_attitude();
        EntangleState get_entangled_status();
        FallState get_falling_status();
};

class StanceDetection : public ArwainJob
{
    TESTABLE:
        void run_test_stance_detector();
        void setup_inference();
        void cleanup_inference();
        void run_inference();
        void run();
        void setup_test_stance_detector();
        void cleanup_stance_detector();
        void run_idle();
        void core_setup();

    TESTABLE:
        ArwainThread job_thread;

        arwain::Logger stance_file;

        // Stance detector object.
        StanceDetector* stance = nullptr;

        std::deque<ImuData> imu_data;
        std::deque<Vector3> vel_data;
        Quaternion rotation_quaternion;

    public:
        StanceDetection();
        bool init();
        bool join();
        StanceDetector::FallState get_falling_state();
        StanceDetector::EntangleState get_entangled_state();
        StanceDetector::Attitude get_attitude();
        StanceDetector::Stance get_stance();
};

StanceDetector::FallState operator|(const StanceDetector::FallState &stance1, const StanceDetector::FallState &stance2);
StanceDetector::EntangleState operator|(const StanceDetector::EntangleState &stance1, const StanceDetector::EntangleState &stance2);

#endif
