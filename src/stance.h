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

#endif
