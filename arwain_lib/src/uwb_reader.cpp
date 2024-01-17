#include <tuple>
#include <string>
#include <sstream>
#include <algorithm>

#include "arwain/arwain.hpp"
#include "arwain/serial.hpp"
#include "arwain/logger.hpp"
#include "arwain/exceptions.hpp"
#include "arwain/thread.hpp"
#include "arwain/transmit_lora.hpp"
#include "arwain/uwb_reader.hpp"

#include "uubla/utils.hpp"

UublaWrapper::UublaWrapper()
{
    init();
}

void UublaWrapper::core_setup()
{
    uubla = new UUBLA::Network;
}

void UublaWrapper::run_idle()
{
    sleep_ms(10);
}

void UublaWrapper::run()
{
    // TODO Currently configured to act as an UUBLA master; need to generalize.
    if (!arwain::config.use_uwb_positioning || arwain::config.node_id != 2)
    {
        return;
    }

    while (mode != arwain::OperatingMode::Terminate)
    {
        switch (mode)
        {
            case arwain::OperatingMode::Inference:
                run_inference();
                break;
            default:
                run_idle();
                break;
        }
    }
}

void UublaWrapper::setup_inference()
{
    uubla->force_plane(true);
    uubla->set_ewma_gain(0.1);
    // uubla->add_node_callback = inform_new_uubla_node; // Replaced with event registrations.
    // uubla->remove_node_callback = inform_remove_uubla_node; // Replaced with event registrations.
    serial_reader_th = std::thread{serial_reader_fn, uubla, "port", 115200};
}

void UublaWrapper::cleanup_inference()
{
    // serial_reader_th.join(); // TODO Make this work; need a way to stop thread
}

void UublaWrapper::run_inference()
{
    setup_inference();
    
    // TODO: Get the framerate (30) from somewhere sensible, or hard code?
    IntervalTimer<std::chrono::milliseconds> timer{1000/30};

    while (mode != arwain::OperatingMode::Terminate)
    {
        uubla->process_queue();
        uubla->solve_map();
        uubla->process_callbacks();
        timer.await();
    }

    cleanup_inference();
}

bool UublaWrapper::init()
{
    core_setup();
    job_thread = ArwainThread{&UublaWrapper::run, "arwain_uubla_th", this};
    return true;
}

bool UublaWrapper::join()
{
    delete uubla;
    while (!job_thread.joinable())
    {
        sleep_ms(1);
    }
    if (job_thread.joinable())
    {
        job_thread.join();
        return true;
    }
    return false;
}

double UublaWrapper::get_distance(const int position)
{
    // TODO Do we still need this function?  Library doesn't support it.
    // return uubla->get_distance(position);
    return 0;
}

Vector3 UublaWrapper::get_own_position() const
{
    // return get_node_position(own_id); // TODO
    return {0, 0, 0};
}

Vector3 UublaWrapper::get_node_position(const int node_id) const
{
    // TODO
    return {0, 0, 0};
}