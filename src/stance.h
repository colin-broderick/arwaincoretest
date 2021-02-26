#ifndef STANCE_H
#define STANCE_H

#include <vector>
#include <array>
#include <chrono>
#include <mutex>
#include <deque>

class StanceDetector
{
    public:
        enum STANCE {
            Inactive,
            Crawling,
            Walking,
            Running,
            Searching,
            Climbing
        };
        enum FALLING {
            NotFalling,
            Falling
        };
        enum ENTANGLED {
            NotEntangled,
            Entangled
        };
        enum AXIS {
            XAxis,
            YAxis,
            ZAxis
        };
        enum ATTITUDE {
            Vertical,
            Horizontal
        };

    private:
        double m_a_mean_magnitude;
        double m_g_mean_magnitude;
        double m_v_mean_magnitude;

        int m_vertical_axis = 1;
        int m_speed_axis;

        int m_count = 0;
        double m_struggle = 0;

        // Status indicators.
        FALLING m_falling = NotFalling;
        ENTANGLED m_entangled = NotEntangled;
        ATTITUDE m_attitude = Vertical;
        STANCE m_stance = Inactive;
        int m_climbing = 0;

        // Fall/entanglment thresholding parameters.
        double m_fall_threshold;
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
        std::mutex fall_lock;
        std::mutex stance_lock;

    public:

        // Constructors.
        StanceDetector(double fall_threshold, double crawling_threshold, double running_threshold, double walking_threshold, double active_threshold, double struggle_threshold);

        // General methods.
        void run(std::deque<std::array<double, 6>> *imu_data, std::deque<std::array<double, 3>> *vel_data);
        AXIS biggest_axis(std::array<double, 3> arr);
        double activity(double a, double g, double v);
        double vector_mean(std::vector<double> values);
        double buffer_mean_magnitude(std::vector<std::array<double, 3>> *buffer);
        double buffer_mean_magnitude(std::deque<std::array<double, 3>> *buffer);
        std::array<double, 3> get_means(std::vector<std::array<double, 3>> *source_vector);
        std::array<double, 3> get_means(std::deque<std::array<double, 3>> *source_vector);
        std::array<double, 6> get_means(std::deque<std::array<double, 6>> *source_vector);

        // Getters.
        STANCE getStance();
        ATTITUDE getAttitude();
        ENTANGLED getEntangledStatus();
        FALLING getFallingStatus();
};

#endif
