#ifndef INDOOR_POSITIONING_WRAPPER_H
#define INDOOR_POSITIONING_WRAPPER_H

#include "vector3.hpp"
#include "arwain.hpp"
#include "corner_detector.hpp"
#include "floor_tracker.hpp"

class IndoorPositioningSystem
{
    TESTABLE:
        void core_setup();
        void setup_inference();
        void cleanup_inference();
        void run_idle();
        void run_inference();
        void run();

        ArwainThread job_thread;
        arwain::Logger corner_log;
        arwain::Logger tracked_floor_log;
        arwain::CornerDetector corner_detector{11, 115.0, 0.20}; // 11 * 0.2 means a window of 11 points separated by at least 20 cm each, so about 2 m total.
        arwain::FloorTracker floor_tracker{5, 0.10, 0.20}; // UK stairs are gradient approx. 0.9.

    public:
        IndoorPositioningSystem();
        void join();
        bool init();

    

    class IndoorPositioningWrapper
    {
        TESTABLE:
            double m_x = 0;
            double m_y = 0;
            double m_z = 0;

            // TODO Create inner IPS object with Ross's rust library.

        public:
            void update(const double &time, const double &x, const double &y, const double &z);
            Vector3 get_position() const;
            double get_x() const;
            double get_y() const;
            double get_z() const;
    };
};

#endif
