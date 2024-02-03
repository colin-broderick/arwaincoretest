#ifndef INDOOR_POSITIONING_WRAPPER_H
#define INDOOR_POSITIONING_WRAPPER_H

#include <thread>

#include <arwain/vector3.hpp>
#include <arwain/logger.hpp>

#include "arwain/corner_detector.hpp"
#include "arwain/floor_tracker.hpp"
#include "arwain/job_interface.hpp"

class IndoorPositioningSystem : public ArwainJob, protected IArwainJobSpec
{
    private:
        void core_setup() override;
        void setup_inference() override;
        bool cleanup_inference() override;
        void run_idle() override;
        void run_inference() override;
        void run() override;

        std::jthread job_thread;
        arwain::Logger corner_log;
        arwain::Logger tracked_floor_log;
        arwain::CornerDetector corner_detector{11, 115.0, 0.20}; // 11 * 0.2 means a window of 11 points separated by at least 20 cm each, so about 2 m total.
        arwain::FloorTracker floor_tracker{5, 0.10, 0.20}; // UK stairs are gradient approx. 0.9.

    public:
        IndoorPositioningSystem();
        bool join() override;
        bool init() override;

    

    class IndoorPositioningWrapper
    {
        private:
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
