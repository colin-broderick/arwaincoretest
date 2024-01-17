#ifndef _ARWAIN_UWB_READER_HPP
#define _ARWAIN_UWB_READER_HPP

#include <uubla/uubla.hpp>

#include "arwain/thread.hpp"
#include "arwain/job_interface.hpp"

class UublaWrapper : public ArwainJob
{
    private:
        void run_inference();
        void run();
        void setup_inference();
        void run_idle();
        void core_setup();
        void cleanup_inference();
        bool init();

    private:
        ArwainThread job_thread;
        ArwainThread solver_th;
        UUBLA::Network* uubla;

    public:
        UublaWrapper();
        bool join() override;
        double get_distance(const int position);
        Vector3 get_own_position() const;
        Vector3 get_node_position(const int node_id) const;
        std::thread serial_reader_th; // TODO Need a way to stop thread - maybe std::stop_token

};

#endif
