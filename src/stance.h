#ifndef STANCE_H
#define STANCE_H

#include <vector>
#include <array>
#include <chrono>

int update(double ax, double ay, double az, double gx, double gy, double gz, double vx, double vy, double vz);
int empty_stance_buffers();
int empty_fall_buffers();
int is_falling();
int is_entangled();
int reset();
double buffer_mean(std::vector<std::array<double, 3>> buffer);
double struggle_mean(std::vector<double> struggles);
int run_fall_detect();
double activity(double a, double g, double v);
int biggest_axis(double arr[3]);
int is_horizontal();
int get_means(double arr[], std::vector<std::array<double, 3>> source_vector);
int run_stance_detect();
int get_stance();
int kill();
int start();

class Stance
{
    private:
        double m_a_mean_magnitude;
        double m_g_mean_magnitude;
        double m_v_mean_magnitude;

        int m_die = 0;
        int m_falling = 0;
        int m_entangled = 0;
        int m_horizontal = 0;
        int m_climbing = 0;
        std::string m_stance = "inactive";

        // Fall/entanglment params.
        double m_a_threshold;
        double m_struggle_threshold;
        double m_gravity;
        double m_a_twitch;
        double m_tmp_struggle;
        double m_sfactor;
        
        double m_act;

        // Stance params.
        double m_climbing_threshold;
        double m_crawling_threshold;
        double m_running_threshold;
        double m_walking_threshold;
        double m_active_threshold;

        int m_vertical_axis = 1;
        int m_speed_axis;

        int m_count = 0;
        double m_struggle = 0;


        std::vector<double> m_struggle_window;

    public:
        // Constructors.
        Stance(double a_threshold, double crawling_threshold, double running_threshold, double walking_threshold, double active_threshold);

        // General methods.
        void run(std::deque<std::array<double, 6>> imu_data, std::deque<std::array<double, 3>> vel_data);
        double activity(double a, double g, double v);
        double vector_mean(std::vector<double> values);
        double buffer_mean_magnitude(std::vector<std::array<double, 3>> buffer);
        double buffer_mean_magnitude(std::deque<std::array<double, 3>> buffer);

        // Getters.
        std::string getStance();
        int is_horizontal();
        int is_entangled();
        int is_falling();

        std::array<double, 3> get_means(std::vector<std::array<double, 3>> source_vector);
        int biggest_axis(std::array<double, 3> arr);
        std::array<double, 6> get_means(std::deque<std::array<double, 6>> source_vector);
        std::array<double, 3> get_means(std::deque<std::array<double, 3>> source_vector);
};

#endif
