#ifndef PREDICT_VELOCITY_H
#define PREDICT_VELOCITY_H

#define USE_SOCKET_INFERENCE 0

#include <chrono>
#include <thread>
// #include <zmq.h>
#include <deque>
#include <array>
#include <sstream>
#include <fstream>
#include <mutex>
#include <string.h>
#include <iomanip>

#include "utils.h"

#if USE_SOCKET_INFERENCE == 0
//#include "arwain_torch.h"
#endif

extern arwain::Configuration CONFIG;
extern int LOG_TO_FILE;
extern int NO_INFERENCE;
extern unsigned int IMU_READING_INTERVAL;
extern std::string FOLDER_DATE_STRING;
extern int SHUTDOWN;
extern std::mutex IMU_BUFFER_LOCK;
extern std::mutex VELOCITY_BUFFER_LOCK;
extern std::mutex POSITION_BUFFER_LOCK;
extern std::deque<std::array<double, 6>> IMU_WORLD_BUFFER;
extern std::deque<std::array<double, 3>> VELOCITY_BUFFER;
extern std::deque<std::array<double, 3>> POSITION_BUFFER;
extern unsigned int VELOCITY_PREDICTION_INTERVAL;

void predict_velocity();

#endif
