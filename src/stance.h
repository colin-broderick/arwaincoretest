#include <vector>
#include <array>
#include <chrono>

// #include "Kernel.h"
// #include "mbed.h"

std::chrono::_V2::system_clock::time_point now();
int update(float ax, float ay, float az, float gx, float gy, float gz, float vx, float vy, float vz);
int empty_stance_buffers();
int empty_fall_buffers();
int is_falling();
int is_entangled();
int reset();
float buffer_mean(std::vector<std::array<float, 3>> buffer);
float struggle_mean(std::vector<float> struggles);
int run_fall_detect();
float activity(float a, float g, float v);
int biggest_axis(float arr[3]);
int is_horizontal();
int get_means(float arr[], std::vector<std::array<float, 3>> source_vector);
int run_stance_detect();
int get_stance();
int kill();
int start();
