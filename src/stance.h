#ifndef STANCE_H
#define STANCE_H

#include <vector>
#include <array>
#include <chrono>
#include <mutex>
#include <deque>

#include "quaternions.h"

namespace arwain
{
    class StanceDetector
    {
        public:
            enum STANCE {
                Inactive,
                Walking,
                Searching,
                Crawling,
                Running,
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

            std::array<double, 3> m_accel_means;
            std::array<double, 3> m_speed_means;

            int m_vertical_axis = 1;
            int m_primary_axis;
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
            AXIS biggest_axis(const std::array<double, 3> &arr);
            double activity(double a, double g, double v);
            double vector_mean(const std::vector<double> &values);
            double buffer_mean_magnitude(const std::vector<std::array<double, 3>> &buffer);
            double buffer_mean_magnitude(const std::deque<std::array<double, 3>> &buffer);
            std::array<double, 3> get_means(const std::vector<std::array<double, 3>> &source_vector);
            std::array<double, 3> get_means(const std::deque<std::array<double, 3>> &source_vector);
            std::array<double, 6> get_means(const std::deque<std::array<double, 6>> &source_vector);
        
        public:
            // Constructors.
            StanceDetector(double freefall_sensitivity, double crawling_threshold, double running_threshold, double walking_threshold, double active_threshold, double struggle_threshold);

            // General methods.
            void update_attitude(quaternion rotation_quaternion);
            void run(const std::deque<std::array<double, 6>> &imu_data, const std::deque<std::array<double, 3>> &vel_data);

            // Getters.
            STANCE getStance();
            ATTITUDE getAttitude();
            ENTANGLED getEntangledStatus();
            FALLING getFallingStatus();
    };
}

arwain::StanceDetector::FALLING operator|(const arwain::StanceDetector::FALLING &stance1, const arwain::StanceDetector::FALLING &stance2);
arwain::StanceDetector::ENTANGLED operator|(const arwain::StanceDetector::ENTANGLED &stance1, const arwain::StanceDetector::ENTANGLED &stance2);

#endif
