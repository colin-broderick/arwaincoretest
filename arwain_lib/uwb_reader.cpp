#include <tuple>
#include <string>
#include <sstream>
#include <algorithm>

#include "arwain.hpp"
#include "serial.hpp"
#include "logger.hpp"
#include "exceptions.hpp"
#include "arwain_thread.hpp"
#include "transmit_lora.hpp"
#include "uwb_reader.hpp"

UublaWrapper::UublaWrapper()
{
    init();
}

void UublaWrapper::core_setup()
{
    uubla = new UUBLA::Network{
        arwain::config.uubla_serial_port,
        arwain::config.uubla_baud_rate
    };
}

void UublaWrapper::run_idle()
{
    sleep_ms(10);
}

void UublaWrapper::run()
{
    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        switch (arwain::system_mode)
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
    uubla->configure("force_z_zero", true);
    uubla->configure("ewma_gain", 0.1);
    uubla->add_node_callback = inform_new_uubla_node;
    uubla->remove_node_callback = inform_remove_uubla_node;
    uubla->start_reading(); // Start as in start reading the serial port. TODO investigate this function for possible leaks.
    solver_th = ArwainThread{solver_fn, "arwain_uub2_th", uubla};
}

void UublaWrapper::cleanup_inference()
{
    uubla->join();
    solver_th.join();
}

void UublaWrapper::run_inference()
{
    setup_inference();
    
    while (arwain::system_mode != arwain::OperatingMode::Terminate)
    {
        sleep_ms(10);
    }

    cleanup_inference();
}

bool UublaWrapper::init()
{
    // TODO Currently configured to act as an UUBLA master; need to generalize.
    if (!arwain::config.use_uwb_positioning || arwain::config.node_id != 2)
    {
        return false;
    }
    core_setup();
    job_thread = ArwainThread{&UublaWrapper::run, "arwain_uubla_th", this};
    return true;
}

void UublaWrapper::join()
{
    delete uubla;
    if (job_thread.joinable())
    {
        job_thread.join();
    }
}

double UublaWrapper::get_distance(const int position)
{
    return uubla->get_distance(position);
}
